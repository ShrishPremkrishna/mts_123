---
date: 2026-10-27T15:13
tags:
  - engineering
  - pupper
---
# Pupper foundation: motions and training

This branch completes the engineering implementation separately from the Lab 6
learning exercises on the student's normal branch. Implemented code, a trained
checkpoint, scored simulation, and successful hardware behavior are different
milestones. The scripts never label a checkpoint hardware-approved.

## Run from a reproducible environment

Install `pupper-mjlab` in its own Python environment. Training needs NVIDIA CUDA;
the motion geometry tests run on a CPU. Colab G4 is the preferred training and
UniMate-generation runtime. The updated notebooks clone the `pupper-foundation`
branch of `ShrishPremkrishna/mts_123`, resolve a single source revision, and import
the implementation from the checkout. Set `PINNED_COMMIT` to reproduce a reviewed
revision exactly. Restart the runtime before changing imported code.

Run these commands from `pupper-mjlab`. Set `PUPPER_OUTPUT` to an **actually
mounted** persistent directory. `/content` on Colab is temporary; creating a
directory named `/content/drive` does not mount Drive. W&B additionally receives
checkpoints, exported policies and the foundation run manifest. Keep API keys in
Colab Secrets or the service's interactive login, never in notebooks or Git.

```sh
export PUPPER_OUTPUT=/your/persistent/foundation
python -m mjlab.scripts.pupper_foundation prepare --output "$PUPPER_OUTPUT"
python -m mjlab.scripts.pupper_foundation train sit_stay_shake \
  --output "$PUPPER_OUTPUT" --num-envs 64 --iterations 2 --logger tensorboard
```

The first training is a smoke test. It checks simulator/PPO/export execution and
does **not** produce a usable policy. After it works, run the actual profiles:

```sh
python -m mjlab.scripts.pupper_foundation train sit_stay_shake --output "$PUPPER_OUTPUT"
python -m mjlab.scripts.pupper_foundation train spider_clips --output "$PUPPER_OUTPUT"
python -m mjlab.scripts.pupper_foundation train spider_walk --output "$PUPPER_OUTPUT"
```

The two tracking profiles use 4,096 environments and 3,000 iterations each;
spider walking uses 4,096 and 4,000. These are course starting budgets, not a
guarantee of convergence. Reward settings live once in
`configs/foundation/training.json`. Spider weights are a starting hypothesis:
low-body support matters, and penalizing abduction toward zero conflicts with
the intended splayed stance. Improve them based on measured failures.

Each run receives a unique directory under `PUPPER_OUTPUT/runs`, printed before
training. It contains the frozen motion, exact environment/agent settings,
checkpoint(s), `policy.json`, and `foundation-run.json`. The manifest captures
source-file hashes, Git revision, package versions, seed, GPU and status. It
explicitly records interrupted/failed runs and requires both checkpoint and
policy export for `trained_unvalidated`. Never deploy a two-iteration smoke
checkpoint. Resuming requires an explicit W&B run; optionally specify an exact
checkpoint rather than its latest one:

```sh
python -m mjlab.scripts.pupper_foundation train sit_stay_shake \
  --output "$PUPPER_OUTPUT" --resume-run entity/project/run-id \
  --resume-checkpoint model_1500.pt --iterations 1500
```

Resuming performs the requested number of **additional** learner iterations.
Use the same source, motion, observation layout and settings; a run ID alone
cannot establish their compatibility. Changing reward or dynamics settings is a
new experiment even if the actor is warm-started.

## What the clip implementation does

- `sit_stay_shake`: sit down → idle loop; either paw shake returns to idle;
  `stand_up` exits. Training includes the idle loop and both action/return edges.
- `spider_clips`: stand → spider down → spider idle loop → spider up. The idle
  joints and root height exactly match the separate spider walking controller.
- Shared helpers keep both cut boundary frames, copy arrays, check finite data,
  mirror all three motor angles while swapping legs, and reverse references.
- Clip assembly checks joint/root continuity, loop/default transitions, unique
  names, frame rates, valid data and training coverage.

The provided raw spider recording is preserved. Its prepared reference clamps
small recorded joint overshoots (maximum about 0.016 rad) to model limits and
raises frames with about 2 mm ground penetration. The final hand-off pose remains
exact. Touching a joint limit is reported separately from exceeding one.

## Evaluate before combining policies

Score the exact checkpoint and frozen motion, not the latest arbitrary file in a
shared log folder:

```sh
python -m mjlab.scripts.pupper_foundation score \
  --motion "$RUN_DIR/sit_stay_shake.npz" \
  --checkpoint "$RUN_DIR/model_2999.pt" \
  --output "$RUN_DIR/evaluation" --suite --stress-profile sit_stay_shake
```

The suite records nominal dynamics, extra mass, shifted centre of mass, and three
seeded randomized-dynamics trials. `--stress-profile` additionally builds one
continuous held-out reference with 15-second idle clips and repeated actions.
Use `spider_clips` for the spider transition policy. Whole-file wrapping in the
course command generator teleports the robot, so `--loops` alone is not an
endurance test; scores now count and label those resets.

JSON includes frame-by-frame root/joint/body errors, joint speeds, simulated motor
torques, aggregate RMS/95th-percentile errors, and the course score. `HOLDS UP` is reference tracking within
3 cm/10°; failure is above 10 cm/45°. The nominal score deliberately removes
several training randomizations. This score does not test temperature, real
contact forces, stop behavior, or hand-offs between distinct controllers.

Use `pupper_locomotion` for walking-policy command/transition evaluation, with
the provided bounded Spider command grid:

```sh
python -m mjlab.scripts.pupper_locomotion evaluate \
  --task Mjlab-SpiderWalk-Flat-Pupper-v3 --checkpoint "$RUN_DIR/model_3999.pt" \
  --commands configs/foundation/spider_commands.json \
  --out "$RUN_DIR/velocity-evaluation" --num-envs 32 --seconds 10 --warmup 2 --seed 101
```
Spider training bounds are 0.15 m/s forward, 0.10 m/s lateral and 0.6 rad/s yaw;
runtime limits must be no higher than measured reliable speeds. Test prolonged
idle, repeated paw requests, stop/cancel, and locomotion ↔ trick ↔ spider changes
in the integration layer. Exporters check neural-network numerical parity;
existing gait parity tests also cover the training/export/deployment reference
contract.

Newly trained trick policies require the course's TA review of scores and viewer
before their first real-robot execution. Keep the physical e-stop available and
observe the course's thermal precautions during supervised hardware validation.
No training or evaluation script here operates the robot.

## Optional 3: text-to-motion

UniMate stays in its own environment because it uses different pinned model
dependencies. Its provided installer pins the source revision and downloads its
released weights. Run it once on the GPU host:

```sh
bash src/mjlab/tasks/tracking/config/pupper/unimate/install.sh /path/to/UniMate
python -m mjlab.scripts.pupper_generate generate --unimate /path/to/UniMate \
  --prompt "An object bows with its chest low and its rear up." \
  --samples 4 --seed 42 --output "$PUPPER_OUTPUT/bow-candidates"
```

For the initial foundation experiment, use a modest standing dance and inspect
all four samples instead of choosing an index blindly. The batch saves every
fit, repair, geometry report, physics trace and actual physics video:

```sh
python -m mjlab.scripts.pupper_generate generate --unimate /content/UniMate \
  --prompt "An object gently sways its body from side to side while standing on all four legs." \
  --samples 4 --seed 42 --output "$PUPPER_OUTPUT/standing-dance-generated"
MUJOCO_GL=egl python -m mjlab.scripts.pupper_unimate_candidates \
  --raw "$PUPPER_OUTPUT/standing-dance-generated/samples.npz" \
  --output "$PUPPER_OUTPUT/standing-dance-candidates" --videos
```

`candidates.json` names a candidate for inspection and explains exclusions.
The standing-dance screen rejects invalid geometry, nearly static motion,
reference teleports, and more than 60 degrees of actual body tilt. Root tracking
drift alone does not disqualify an RL candidate. Nominal contact, torque, height
and tilt traces remain evidence to inspect; this heuristic is not physical
approval. Train the selected NPZ with `generated_trick` only after reviewing
its video, then run the same held-out dynamics score suite used for other tricks.
Generation also records the UniMate revision, weights hashes and its separate
environment snapshot. Raw generations and before-repair references are retained.

Inspect candidates and choose one explicitly. Fitting records raw-sample and
source hashes and saves both the original fit and repaired reference:

```sh
python -m mjlab.scripts.pupper_generate fit \
  --raw "$PUPPER_OUTPUT/bow-candidates/samples.npz" --sample 0 --name secret_bow \
  --output "$PUPPER_OUTPUT/bow-fit"
python -m mjlab.tasks.tracking.config.pupper.play_motion \
  "$PUPPER_OUTPUT/bow-fit/secret_bow.npz"
python -m mjlab.scripts.pupper_foundation train generated_trick \
  --motion "$PUPPER_OUTPUT/bow-fit/secret_bow.npz" --output "$PUPPER_OUTPUT"
```

Automatic cleanup is only a starting repair: grounded standing entry/exit,
contact anchoring with hysteresis, IK, floor clearance, and uniform slowdown.
It exposes unreachable targets and other geometry failures instead of declaring
arbitrary generated motion feasible. A bad sample may need different prompts,
timing, or a motion-specific repair. Physics replay and trained-policy evaluation
remain necessary. The model worker has bounded response waits and cleans up its
process and descriptors on failure. Close it before RL to free GPU memory.

## Simulator and deployment target limits

The provided training action sends `default + scale * action` without target
clipping. The robot controller clips that target to joint limits. A policy can
therefore learn forces against a simulated soft joint stop that its deployed
motor command will not reproduce. Keep legacy results and compare them with the
actual deployment contract explicitly:

```sh
python -m mjlab.scripts.pupper_foundation score --motion exact-motion.npz \
  --checkpoint exact-model.pt --output "$PUPPER_OUTPUT/deploy-clamp-evaluation" \
  --suite --deployment-target-clamp
python -m mjlab.scripts.pupper_foundation train sit_stay_shake \
  --initial-checkpoint exact-model.pt --deployment-target-clamp --iterations 1000 \
  --output "$PUPPER_OUTPUT"
```

The opt-in clips the absolute processed target while preserving the policy's raw
previous-action observation, matching C++. It does not change task defaults or
old runs. New run names and manifests state `deployclamp` versus `legacytargets`;
local warm-start bytes are copied into the new run. Locomotion train/evaluate
commands accept the same flag. Evaluation reports how often an unbounded target
would exceed limits, even while the actual target is clipped. The flag alone
does not establish transfer: compare tracking, falls, slip and effort, then
validate the resulting model under the same contract and held-out conditions.

## Local verification

```sh
python -m pytest tests/test_pupper_foundation.py tests/test_pupper_tracking.py \
  tests/test_pupper_spider.py tests/test_pupper_unimate.py \
  tests/test_unimate_repair.py tests/test_unimate_worker.py \
  tests/test_foundation_evaluation.py tests/test_unimate_candidates.py -q
```

These tests cover real geometry/mirroring, clip transitions and exported arrays,
invalid inputs, source preservation, repair behavior, and worker cleanup/timeouts.
They do not train policies. GPU and hardware results must be recorded separately.

### Controlled tracking continuations

To compare a deployment-clipped continuation with its original policy, retain the
exact exported reference and checkpoint. The launcher respects an explicit
`--motion` even for built-in profiles and copies those bytes into the new run.
For example, from the project directory on a CUDA host:

```bash
python -m mjlab.scripts.pupper_foundation train spider_clips \
  --motion /persistent/original/spider_walk.npz \
  --initial-checkpoint /persistent/original/model_2999.pt \
  --deployment-target-clamp --iterations 1000 --seed 42 \
  --output /persistent/spider-clamp-continuation
```

This changes only the target actuation contract. Evaluate against the original
policy with the same reference, seed and deployment clamp before changing the
reward or motion. `--compose-com-randomization` is a separate opt-in: the course
inertial randomization otherwise erases the already sampled body COM offset.
The composition preserves the RNG draw order and the mass, friction, encoder and
gain samples, while retaining the intended COM shift. Run identities and manifests
record both choices. Old policies and default training configurations stay intact.

Physics video output also writes a `.trajectory.npz` file before rendering. It
contains the recorded robot state and aligned reference at every physics control
step, so video rendering can be retried without rerunning or altering the rollout.
Rendering streams batches to the video encoder rather than retaining the entire
video in memory.
