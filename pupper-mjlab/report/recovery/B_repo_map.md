# B. Repo map for CS123 Optional Lab 2 (Pupper, `Mjlab-MixedGaits-Flat-Pupper-v3`)

Surveyed 2026-11-18. Read-only. Nothing was modified or pushed.

Abbreviations used below:
- **U**: the upstream course repo `cs123-stanford/pupper-mjlab` at HEAD `aa475e9`, cloned to `/tmp/claude-1000/-home-ubuntu/1e6973a7-fdc0-4a4d-af73-a538d53a1bf0/scratchpad/src/pupper-mjlab`. The history was deepened to all 11 commits. `gait_reference.py` was last changed in 52e61ce on 2026-10-19.
- **F**: the student's monorepo `mts_123` at `59db01658a` (2026-11-04), with paths under `pupper-mjlab/`. It is checked out as a worktree at `/tmp/claude-1000/-home-ubuntu/1e6973a7-fdc0-4a4d-af73-a538d53a1bf0/scratchpad/src/mts_123_59db016`.
- **M**: `mts_123` `main` (`50e8e15`), cloned to `.../scratchpad/src/mts_123`. It is 16 commits after 59db016.

## 0. Key finding: `train` is not the `train` console script

The `train --out ... --profile ... --weights ... --reference-dir ...` command line is a **subcommand of F's `src/mjlab/scripts/pupper_locomotion.py`**. It is run as:
```
python -m mjlab.scripts.pupper_locomotion train ...
```
The `train` console script in `pyproject.toml:63` maps to `mjlab.scripts.train:main`. That script is upstream mjlab's tyro CLI: `uv run train <TASK> --env.scene.num-envs N ...`. It does **not** accept `--out`, `--profile` and the other custom flags. `pupper_locomotion.py` does not exist in U.

## 1. Install and versions

U and F have byte-identical `pyproject.toml` and `uv.lock`. The project is `mjlab` 1.5.2 and needs Python `>=3.10,<3.14`.

**Pinned versions in `uv.lock`:**

| Package | Version |
|---|---|
| torch | 2.9.0+cu128 (extra `cu128`, index `download.pytorch.org/whl/cu128`) |
| warp-lang | 1.14.0 (index `pypi.nvidia.com`) |
| mujoco | 3.10.0 |
| mujoco-warp | 3.10.0.2 |
| rsl-rl-lib | 5.4.0 |
| numpy | 2.2.6 / 2.3.4 (capped below 2.5) |
| wandb | 0.22.3 |
| viser | 1.0.27 |
| mjviser | 0.0.14 |
| tyro | 1.0.1 |
| nvidia-cuda-runtime-cu12 | 12.8.90 |

There is **no JAX**: the stack is MuJoCo-Warp plus torch.

**Makefile targets (U = F):**
- `make sync`: `uv sync --all-packages --extra cu128 --group dev`
- `make sync-cpu`
- `make format`: ruff
- `make type`: ty and pyright
- `make check`: format and type
- `make test`: pytest
- `make test-fast`: `-m "not slow"`
- `make test-cpu` and `make test-cpu-fast`: `FORCE_CPU=1`
- `make build`
- `make docs`
- `make docker-build`

**How the old Colab setup worked:** `foundation/scripts/colab_bootstrap.py:20-22` ran `uv sync --extra cu128` in `pupper-mjlab/`, then `uv run --no-sync python -c 'import torch...'`. The worker set `MUJOCO_GL=PYOPENGL_PLATFORM=egl` (`colab_optional_launch.py:31`).

**Console scripts (`pyproject.toml:62-69`):**
- `train`
- `play`
- `demo`
- `list-envs`
- `viz-nan`
- `export-scene`
- `export-pupper-policy`

**Module entrypoints:**
- `python -m mjlab.tasks.pupper_gait.visualize_reference --gait {trot|lift|reach|...} [--gravity] [--viser True --share True]`
  - This is a tyro CLI (`visualize_reference.py:33-44`). With `--share` it prints a viser share URL (`:103-105`).
  - In U, `--gait lift` raises `NotImplementedError` until the TODO block is filled. In F, lift is filled.
- `python -m mjlab.tasks.pupper_gait.bootstrap {fold|design|jump|compare}`: F only.
- `python -m mjlab.scripts.pupper_locomotion {train|evaluate}`: F only.
- `notebooks/watch_live.py <TASK> [--url-file P] [--any-checkpoint]`: a live viser **share-URL** viewer that hot-swaps checkpoints (`watch_live.py:76-77`). It is present in U and F.
- `uv run play <TASK> --checkpoint-file X.pt --viewer viser|native [--agent zero|random] [--num-envs N]`: `play.py:34-50`.
- `uv run export-pupper-policy <TASK> --checkpoint-file X.pt --output policy.json` (or `--wandb-run-path`, `--upload-wandb`).
  - The PupperOnPolicyRunner also auto-uploads `policy.json` to W&B on save and finish.

### Smoke test (F, on the A100)

These commands are **not executed**; nothing was installed during the survey.

**1. Install and check the environment:**
```bash
cd <mts_123 checkout @59db016>/pupper-mjlab
~/.local/bin/uv sync --locked --extra cu128            # add --group dev for pytest
uv run --no-sync python -c "import torch,warp,mujoco,mujoco_warp;print(torch.__version__,torch.cuda.is_available(),warp.__version__,mujoco.__version__)"
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
PHYS="--deployment-target-clamp --compose-com-randomization --locomotion-torso-contact --driver-limits ../foundation/configs/class-driver-joint-limits.json"
```

**2. Train.** `--out` must NOT exist yet (`mkdir(exist_ok=False)`):
```bash
uv run --no-sync python -m mjlab.scripts.pupper_locomotion train --out /tmp/smoke/base --profile baseline --iterations 3 --num-envs 64 --seed 42 --logger tensorboard $PHYS
ls /tmp/smoke/base     # provenance.json params/ randomization_snapshot.npz model_*.pt
```

**3. Warm-start from the shipped authored reference:**
```bash
uv run --no-sync python -m mjlab.scripts.pupper_locomotion train --out /tmp/smoke/ref --profile baseline --reference-dir configs/optional2/extended_trot --checkpoint /tmp/smoke/base/model_<last>.pt --iterations 2 --num-envs 64 --seed 42 --logger tensorboard $PHYS
```

**4. Evaluate collector infrastructure with a zero policy:**
```bash
echo '[[1.5,0,0],[-1.5,0,0]]' > /tmp/smoke/cmds.json
uv run --no-sync python -m mjlab.scripts.pupper_locomotion evaluate --zero-policy --out /tmp/smoke/eval0 --num-envs 8 --seconds 3 --warmup 1 --commands /tmp/smoke/cmds.json --reference-dir configs/optional2/extended_trot $PHYS
```

**5. Part 1 authored gait (no GPU needed) and unit tests:**
```bash
uv run --no-sync python -m mjlab.tasks.pupper_gait.bootstrap design /tmp/smoke/design   # defaults = extended_trot params
uv run --no-sync pytest tests/test_pupper_bootstrap.py tests/test_pupper_gait_export.py -m "not slow"
```

**Upstream path, for comparison.** In U, every reward weight ships at **0**. The notebook sets weights through the Python API (`CS123_Pupper_mjlab.ipynb` cell 17: `load_env_cfg` → set `.weight` → `launch_training(TASK, TrainConfig(...))`). The raw CLI is `uv run train Mjlab-MixedGaits-Flat-Pupper-v3 --env.scene.num-envs 64 --agent.max-iterations 3`, which learns nothing because all weights are zero.

## 2. The `pupper_locomotion` CLI at 59db016 (F only)

The CLI is defined in F `src/mjlab/scripts/pupper_locomotion.py:320-358` (argparse).

**Common flags (train and evaluate), `:324-334`:**
- `--task` (default `Mjlab-MixedGaits-Flat-Pupper-v3`)
- `--reference-dir`
- `--checkpoint`
- `--out` (required)
- `--seed` (default 42)
- `--device` (default `cuda:0`)
- `--deployment-target-clamp`
- `--driver-limits JSON` (requires `--deployment-target-clamp`)
- `--target-slew-rad-s` (not used in the reviewed runs)
- `--encoder-observation-mode {legacy_true,deployment_measured}`
- `--allow-encoder-observation-mode-change`
- `--compose-com-randomization`
- `--locomotion-torso-contact`

**Train-only flags, `:335-337`:**
- `--num-envs` (default 4096)
- `--iterations` (default 3000)
- `--profile {baseline,fast}` (default baseline)
- `--weights JSON`
- `--logger {wandb,tensorboard}` (default wandb)

**Evaluate-only flags, `:338-346`:**
- `--num-envs` (default 32)
- `--seconds` (default 10)
- `--warmup` (default 2)
- `--commands JSON`
- `--randomized`
- `--reset-each-command`
- `--torso-contact-audit`
- `--nominal-mechanics`
- `--no-heading-hold`
- `--allow-out-of-range`
- `--zero-policy`
- `--geometry-audit`
- `--geometry-audit-stride`

**Profiles, `:18-21`:**
- `BASE_WEIGHTS` = `{track_linear_velocity:10, track_yaw_velocity:7, gait_tracking:10, upright:1, lin_vel_z_l2:-0.5, ang_vel_xy_l2:-0.05, action_rate_l2:-0.02, knee_ground_contact:-1}`.
  - This is Shrish's Lab 4 StableGait set. It is identical to `foundation/configs/lab4-stablegait-original-rewards.json`.
- `FAST_WEIGHTS` = BASE plus `track_linear_velocity:15, gait_tracking:5, roll_asymmetry:-2, foot_slip:-0.1, joint_torques_l2:-1e-4`. The code labels these as "untuned hypotheses".

**`--weights` format (`:130-139`):**
- A flat JSON object `{reward_name: float}`. It **replaces** the profile (it does not merge with it).
- Every reward term is first set to 0.
- An unknown name raises `ValueError`, as does a non-finite value or an all-zero set.
- Mystery tasks require `--weights`.
- An example is `configs/optional2/mystery_experiment.json`.

**`--checkpoint` warm start:**
- `:140-143` pops `events.hold_base_airborne` and `curriculum.pretrain_rewards`, so the 500-iteration airborne pretrain is skipped.
- `:160` calls `runner.load(...)`, which restores the full runner and optimizer state. Iteration numbering continues, so 3000 + 1500 iterations gives `model_4499.pt`.

**Run output:**
- `:145`: `agent.experiment_name="pupper_foundation_optional2"` and `run_name=out.name`. The W&B project is the default `"mjlab"` (`rl/config.py:107`).
- `:146-148`: `provenance.json` (args, git rev, source sha256, reference sha256s), plus `params/env.yaml` and `params/agent.yaml`.

**`--reference-dir` (`:348-351`):** the path is resolved and must be a directory. The CLI sets `os.environ["PUPPER_REFERENCE_DIR"]` **before** the task modules are imported, and `gait_reference.py` reads it at import time. Any other process that loads the reference tables needs the same environment variable, e.g. `PUPPER_REFERENCE_DIR=/abs/dir uv run export-pupper-policy ...`.

**Physics flags in `settings()` (`:82-119`):**
- `deployment_contract.apply_deployment_target_clamp`
- `apply_driver_target_clamp(cfg, json)`
- `compose_pupper_com_randomization`
- `locomotion_contact_contract.enable_locomotion_torso_contact`

These modules live in F `src/mjlab/tasks/pupper/`: `deployment_contract.py`, `locomotion_contact_contract.py`, `target_slew.py`, `encoder_observation.py`, `randomization_audit.py`, `collision_audit.py` and `torso_support.py`. The driver limits file is `foundation/configs/class-driver-joint-limits.json`. It sets `_1` to [-1.12, 2.41] (sign-mirrored for left legs), `_2` to [-0.32, 3.04], `_3` to [-2.69, 0.61], with `installed_robot_verified: false`.

**`evaluate` (`:168-317`):**
- Commands are frozen by overwriting `vel_command_b` each step. Heading hold stays on unless `--no-heading-hold` is passed.
- `DEFAULT_SEQUENCE` (`:17`) has 16 segments: `0, .3, .49, .51, 1.0, 1.5, .51, .49, 0, -.3, -1.5, yaw+1, yaw-1, vy+.3, vy-.3, 0`. Commands outside the task range are rejected.
- The run writes `trace.npz`, `metrics.json` (from `bootstrap.summarize_trace`) and `provenance.json`.
- The phase written to the trace (`:271-275`) uses `freq * _GAIT_FREQ_MULT["reach"]` when `|vx| >= 0.5`. That multiplier comes from the reference-dir manifest, so **evaluate must use the same `--reference-dir` as training** if the trace will later be folded.

## 3. `gait_reference.py` (`src/mjlab/tasks/pupper_gait/mdp/`)

### Parametric pipeline (identical in U and F except for lift and the socket)

The pipeline runs in this order:
1. `_base_cycle(stride, n_swing, stance_z, swing_lift)` (U:278-312 / F:298) builds 5 stance keyframes, `x = front, .5f, 0, -.5b, -back` at `z = stance_z`. It then adds `n_swing` swing keyframes.
   - With one keyframe, that keyframe is `[0, 0, z + lift]`, giving duty 4/6.
   - With several, they follow a sine arc.
2. `_interpolate_triangle` (U:315) linearly interpolates the looping keyframes.
3. Each leg reads phase `(t - touchdown[leg]) % 1`. The per-leg `_HIP_OFFSET` is added: FR (.06, -.09), FL (.06, .09), BR (-.11, -.09), LB (-.11, .09) (U:132). Optional `_GAIT_X_SHIFT` and `_GAIT_Y_SHIFT` are also added.
4. `_ik_single_leg` (U:325-343) is finite-difference gradient descent with `lr=10`, `max_iter=100`, `tol=1e-4` and `eps=1e-3`. Each solve is warm-started from the previous phase.
5. The table has shape (100, 12), with leg order FR, FL, BR, LB and joints `_1`, `_2`, `_3`.

**Joint-limit clamp:**
- `_clamp_to_joint_limits` (U:349) is applied only to the jump and spider tables inside `gait_reference`.
- Every table is clamped again in torch by `mdp/gait.py:_reference_table` (U:74-87).
- F adds `_validate_capture` (F:370-378), which **rejects** captured tables that are out of limits instead of clipping them.

### Shipped gaits

| Gait | Values | Location |
|---|---|---|
| trot | touchdown (0, .5, .5, 0); half-stride 0.05; stance_z -0.14; swing lift 0.09; 1 swing keyframe | U:155-198, F:163-204 |
| lift (U) | all `None`; this is the student TODO | U:236-239 |
| lift (F) | touchdown (0, .5, .5, 0); stride 0.0; stance -0.14; lift 0.09 (Lab 4 run 4hkyvuxh) | F:256-259 |
| spider | joint-space sine bump, touchdown (0, .5, .5, 0) | U:249-275 |

**gallop and cheetah are not shipped.** Only comments describe them (U:142-154). They have no table entries. Any gait name not in `_GAIT_TOUCHDOWN` (gallop, cheetah, reach, reach_back without a capture file) **falls back to the trot** (U:397-405). `pupper_gait_env_cfg.py:62-78` explains why the gallop was disabled: it needed 1.44 rad per 20 ms step, about 7.9 Nm against the **3 Nm** effort limit (`pupper_constants.py:86 PUPPER_EFFORT_LIMIT=3.0`).

### Fast-branch constants (identical in U and F; `pupper_gait_env_cfg.py`)

- `GAIT_CYCLE_S = 0.75`, so `GAIT_FREQUENCY` is 1.333 Hz (:49-50). `GAIT_BLEND_SPEED = 0.1` (:54). `GAIT_N_SAMPLES = 100` (:57).
- **Speed threshold:** `TROTGALLOP_ONSET = 0.5` m/s (:120). The fast branch is `|vx| >= 0.05 (_DIR_THRESHOLD, gait.py:46) and |vx| >= 0.5` (`mixed_gaits.py:72-101`). The forward/backward split uses `vx >= 0`: reach for forward, reach_back for backward. Lift is used when not translating.
- **±1.5 m/s cap:**
  - `MIXED_MAX_LIN_VEL_X = 1.5` (:129), applied in `pupper_mixed_gaits_env_cfg` (:487).
  - The exporter's `run_speed_cap: 1.5` and `walk_speed_cap: 0.49` live in `pupper_gait/export.py:298-299`.
  - Fall termination is 45 degrees (:135).
- **Frequency multiplier:**
  - `_GAIT_FREQ_MULT` is **empty** in U (U:202), so the fast clock runs at 1× unless the student edits it.
  - In F (F:209-221), the multiplier is set from `reference_manifest.json["frequency_mult"]` for both reach and reach_back.
  - The phase is computed in `mixed_gaits.py:76-82`. Backward and negative-yaw commands use the reversed phase (`:85-86`).
- `GAIT_GALLOP_FREQ_MULT = 2` (:86) applies only to the single-gait gallop tasks. `GAIT_GALLOP_SPEED = 1e3` disables them.

### Capture socket: what each repo loads

**U:**
- Fixed paths next to the module: `mdp/captured_reach_gaits.npz` with keys `reach_fwd` and `reach_back`, and `mdp/captured_jump_gait.npz` with keys `jump` and `duration_s` (U:37-49).
- **No capture files ship.** The comments refer to run al7sdood, but the file is absent.
- There is no validation and no manifest.

**F:**
- Directory: `$PUPPER_REFERENCE_DIR`, defaulting to the module directory (F:34).
- `captured_reach_gaits.npz` has keys `reach_fwd` and `reach_back`, each of shape (N≥8, 12) in robot order. It also holds optional `joint_names` (checked against `JOINT_NAMES`) and `metadata_json`.
- `reference_manifest.json`:
  - Required if present: `reference_sha256` must equal the sha256 of the npz, otherwise it raises.
  - It sets `frequency_mult`.
  - It also records `version: 1`, `joint_order` and `checks` (from `bootstrap.save_references`, `bootstrap.py:154-165`).
- `captured_jump_gait.npz` has keys `jump`, `duration_s` (0.2 < d < 1.44), `touchdown_s` (0.15 < t ≤ d) and `joint_names`.

**Resampling:**
- Periodic captures use linear interpolation onto 100 points with wrap (U:380-388 / F:421-427).
- The jump capture is resampled without wrap, endpoint to endpoint (U:367-372).
- **`reach_back` must be stored in forward chronological order.** The loader applies `np.roll(table[::-1], 1)` (F:433) to cancel the backward phase reversal. Do not reverse it a second time.

### Bootstrap tools (F `src/mjlab/tasks/pupper_gait/bootstrap.py`)

**`design OUT`** (`:353-356`, `:367-377`) is Part 1, the authored fast gait. Defaults:
- `--half-stride .08`
- `--stance-z -.12`
- `--swing-lift .05`
- `--swing-keyframes 5`
- `--frequency-mult 2`
- touchdown (0, .5, .5, 0)

reach_back is the reversed table. The output is checked against joint limits. These parameters produced `configs/optional2/extended_trot/`, whose reference has a peak joint velocity of 30.99 rad/s.

**`fold TRACE OUT`** (`:349-352`, `:361-366`) is Part 2, folding captured cycles into a reference. Options:
- `--speed 1.5`
- `--samples 100`
- `--min-cycles 3`
- `--harmonics 6`
- `--symmetry`
- `--frequency-mult 1` (**pass 2 to keep the 2× clock**)

How it works (`fold_cycles`, `:64-97`):
- It keeps only samples where `valid` is set and the command is within 0.025 of ±speed.
- It splits cycles at phase wraps, requires a single episode and no failures, and interpolates each cycle onto the phase grid.
- It takes the median, then applies FFT smoothing to 6 harmonics. `mirror_average` is applied only with `--symmetry`.

**`compare BASE.json CAND.json OUT`** (`:270-338`) applies a per-seed promotion rule:
- High-speed segments (|vx| ≥ 0.5) must improve by at least 5%.
- There must be no additional failures.
- Torque², slip and yaw error may regress by at most 20%.

**`jump`** is for the Mystery task.

The full loop from `configs/optional2/README.md:44-50` is train → evaluate → compare → fold → retrain. Per that README, stop after two non-improving rounds.

## 4. MixedGaits reward vocabulary and PPO

The reward vocabulary is the same in U and F because `pupper_env_cfg.py` and `pupper_gait_env_cfg.py` are identical. In U, every weight ships at 0.0 (`pupper/pupper_env_cfg.py:378-500`).

**Inherited and re-parameterized terms:**
- `track_linear_velocity` (std √0.1)
- `track_yaw_velocity`
- `upright`
- `action_rate_l2`
- `air_time`
- `foot_slip`
- `orientation_l2`
- `lin_vel_z_l2`
- `ang_vel_xy_l2`
- `joint_torques_l2`
- `joint_acc_l2`
- `termination`
- `stand_still_pose`
- `stand_still_joint_velocity`
- `abduction_angle`
- `self_collision_r`
- `self_collision_l`
- `knee_ground_contact`
- `base_height` (target 0.145, std 0.05; relaxed above the onset)

**Terms added for gait tasks:**
- `gait_tracking`: `mixed_gait_tracking`, with std `{_1: .15, _2: .3, _3: .3}` and phase lead `1 + latency/decimation`.
- `track_angular_velocity` (std 0.7071)
- `roll_asymmetry` (tau 0.3; MixedGaits only)

**Removed terms:**
- `pose` (popped by `_add_gait`)
- `dof_pos_limits`, `foot_clearance`, `foot_swing_height`, `soft_landing`, `body_ang_vel`, `angular_momentum`

**Weights in each repo:**
- U: all 0. The notebook supplies weights and silently skips unknown names.
- F at 59db016: `BASE_WEIGHTS` or `FAST_WEIGHTS` as listed in section 2. Unknown names raise an error.

**PPO (U = F; `tasks/pupper/rl_cfg.py:15-50` and `pupper_gait/rl_cfg.py`):**
- Actor and critic MLPs are (256, 128, 128, 128), ELU, with obs normalization. The Gaussian init std is 1.0 (scalar).
- `value_loss_coef` 1.0, clipped value loss, `clip_param` 0.2, `entropy_coef` 0.01.
- 5 epochs, 4 minibatches, `lr` 1e-3 with the adaptive schedule.
- `gamma` 0.97, `lam` 0.95, `desired_kl` 0.01, `max_grad_norm` 1.0.
- **`num_steps_per_env` = 24** and **`save_interval` = 50**.
- `max_iterations` is 20000 for gait tasks; `--iterations` overrides it.
- `clip_actions` is None.
- The experiment name is `pupper_mixed_gaits`, which `pupper_locomotion` overrides.

**Timing and pretraining:**
- Physics dt is 0.004 and control dt is 0.02 (50 Hz).
- Episodes last 10 s.
- The airborne pretrain lasts 500 iterations at a height of 0.30 (`pupper_gait_env_cfg.py:157-160`) and is removed on warm start.
- `push_robot` is disabled (`pupper_env_cfg.py:302`).

## 5. What 59db016 contains for opt-lab-2, and what `main` adds

**Present in F:**
- `pupper-mjlab/configs/optional2/README.md`: the full runbook, results table and protocol.
- `configs/optional2/extended_trot/{captured_reach_gaits.npz,reference_manifest.json}`: the authored Part 1 reference.
- `configs/optional2/mystery_experiment.json`.
- `configs/foundation/{training,stable_commands,spider_commands}.json`.
- The code listed in sections 2 and 3, plus tests `tests/test_pupper_bootstrap.py`, `test_deployment_target_clamp.py`, `test_driver_target_clamp.py`, `test_foundation_training_contract.py` and others.
- `foundation/scripts/`:
  - `colab_optional_corrected.py`: resumable flat, validate, bumpy and mystery batches.
  - `colab_optional.py`: the legacy batch.
  - `colab_optional_validation.py`
  - `bumpy_checked_cli.py`: 0.10 m Perlin grid with an overflow audit.
  - `mystery_coverage_cli.py`
  - `colab_bootstrap.py`
  - `colab_optional_launch.py`
- `foundation/results/optional2-*.json`: 12 review records with numbers, e.g. the corrected flat review shows mean xy error going from 0.120 to 0.071 m/s.

The checkpoints themselves lived in `pupper-foundation-artifacts/...` outside the repo and are **lost**. W&B is the only remaining source for them.

**Absent in both F and M:**
- `configs/opt-lab-2/`, top-level `scripts/`, `report/` and `artifacts/` directories.
- Any multi-gait **distillation** code (Part 3).
- A "384 robot-runs" fall-count harness.
- Torque-saturation at 3 Nm, splay or push-test evaluators.
- A dedicated share-viewer script beyond `watch_live.py` and `visualize_reference --share`.

What does exist for evaluation is `pupper_locomotion evaluate` plus `summarize_trace` (`bootstrap.py:100-151`). Per segment it reports:
- failures and timeouts
- mean and RMS xy error, yaw error, achieved vx
- mean torque² and **peak |torque|** (with no 3 Nm saturation fraction)
- contact slip
- **heading drift** (zero-yaw segments only)
- base height
- **peak foot contact force**
- target-clip fractions
- airborne fraction

**What `main` changes:**
- `main` adds nothing relevant to opt-lab-2.
- In `pupper-mjlab`, `main` only changes `AGENTS.md`, `CLAUDE.md` and `.claude/` (it removes `settings.json` with its uv/make allow-list and ruff-format hook).
- The other 16 commits cover `final_project/pupper_fc`, Pi/hardware runtime, `pupper_gait_deploy` C++ safety and foundation results.
- `git diff 59db016 main -- pupper-mjlab/src` is empty.

## 6. Agent conventions (CLAUDE.md → AGENTS.md)

**59db016 root `AGENTS.md`:**
- Start with `python3 foundation/scripts/session_status.py`.
- `foundation/results/interruption-recovery.json` is the canonical record of current state.
- State your assumptions and how you will verify them. Make surgical edits.
- One agent owns each changed file, and the lead integrates.
- Offline work never moves the robot. Simulate any changed motion first.
- Preserve the exact source, configuration, policy and outcomes.
- Long jobs need a verified checkpoint and a verified final archive.
- Keep credentials out of source, logs and artifacts.

**59db016 `pupper-mjlab/CLAUDE.md`:**
- Keep changes scoped to training, rewards, configs and evaluation.
- For each change, name the hypothesis, the comparator and the metric.
- Verify logic with tests.
- Keep simulation evidence separate from physical evidence.

**`main` root `AGENTS.md`:**
- Speed, altitude, depth and range principles.
- Update context records with material progress.
- Keep the environment reproducible: update pins and setup instructions.
- Inspect the branch and diff before editing. Do not rewrite shared history.
- Preserve joint and current limits unless a reviewed change justifies different values.
- Always use `uv run`, never bare python, per the U README.
- Run `make check` before committing.

**`main` `pupper-mjlab/AGENTS.md`:** dependencies are pinned in `uv.lock`, and you should check what a command does before running it against the shared environment.
