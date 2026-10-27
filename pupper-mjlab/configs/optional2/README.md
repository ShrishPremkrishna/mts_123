---
date: 2026-10-27T15:16
tags: [cs123, foundation]
---
# Optional Lab 2: controlled reference bootstrapping

This supplies the missing capture/fold/evaluate pipeline, a joint-limit-checked authored candidate, direction-split reference artifacts, reproducible training, and a conservative promotion comparison. A trained fast policy and robot speed result must still be obtained with the commands below. Source: [Optional Lab 2](https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html), October 27, 2026.

The saved `extended_trot/` candidate uses 0.08 m half-stride, -0.12 m stance, 0.05 m swing lift, five swing keyframes, diagonal timing and a 2× fast-reference clock. A more extreme initial candidate exceeded joint limits and was rejected; this one passes those bounds. Its maximum reference joint speed is in its manifest. Kinematic feasibility alone does not establish low torque, high speed, or hardware safety. The reference has **not** earned promotion over the stock trot.

## First establish the baseline

From a Linux/NVIDIA environment with this exact repository revision installed (`pip install -e ./pupper-mjlab`), run from the outer repository:

```sh
python -m mjlab.scripts.pupper_locomotion train --out artifacts/optional2/baseline --profile baseline --iterations 3000 --num-envs 4096
python -m mjlab.scripts.pupper_locomotion evaluate --checkpoint artifacts/optional2/baseline/model_2999.pt --out artifacts/optional2/baseline-eval --seed 101
```

The baseline weights restore Shrish's recorded Lab 4 eight-term configuration; all unspecified terms are zero. This is a new **MixedGaits baseline**, not a reproduction of the old StableGait run under its older source version. Confirm the actual checkpoint filename after training. Every command writes exact arguments, hashes and a snapshot of environment settings. W&B logging is default; `--logger tensorboard` remains available offline.

## Compare the authored reference, then bootstrap

```sh
python -m mjlab.scripts.pupper_locomotion train --out artifacts/optional2/authored --profile baseline --reference-dir pupper-mjlab/configs/optional2/extended_trot --checkpoint artifacts/optional2/baseline/model_2999.pt --iterations 1500
python -m mjlab.scripts.pupper_locomotion evaluate --checkpoint artifacts/optional2/authored/model_4499.pt --reference-dir pupper-mjlab/configs/optional2/extended_trot --out artifacts/optional2/authored-eval --seed 101
python -m mjlab.tasks.pupper_gait.bootstrap compare artifacts/optional2/baseline-eval/metrics.json artifacts/optional2/authored-eval/metrics.json artifacts/optional2/comparison.json
python -m mjlab.tasks.pupper_gait.bootstrap fold artifacts/optional2/authored-eval/trace.npz artifacts/optional2/round1-reference --frequency-mult 2
python -m mjlab.scripts.pupper_locomotion train --out artifacts/optional2/round1 --profile baseline --reference-dir artifacts/optional2/round1-reference --checkpoint artifacts/optional2/authored/model_4499.pt --iterations 1500
```

**Checkpoint numbers above are examples, not an assumption about the runner's resume numbering.** Use each run's actual final checkpoint. Warm starts load the previous runner state and remove the initial supported-body pretraining phase. Use the same reward weights for the first reference-only ablation. `--profile fast` offers explicit **untuned hypotheses** (more tracking, less imitation, asymmetry/slip/torque penalties); vary these in a separate comparison so gains are attributable. A custom `--weights rewards.json` must name real reward terms. Never train with all-zero defaults.

A reference directory is set before task registration and its cadence and table hashes travel together. The exporter reads the same tables and frequency as training. Backward captures are stored in forward chronological phase; the existing loader reverses them once to compensate for backward phase lookup. Do not reverse them a second time in the capture script.

## What evaluation actually measures

Default commands sweep stop → slow forward → either side of the 0.5 m/s fast threshold → 1.0 and 1.5 m/s → back down, then reverse, turn and sidestep. It keeps the same rollout across command changes by default; `--reset-each-command` is a distinct protocol. Supply a JSON list of `[vx, vy, yaw]` using `--commands` for another task. SpiderWalk requires its own smaller command range; out-of-range commands are refused unless explicitly requested.

`trace.npz` preserves robot-order positions, reference phase, raw/effective commands, episode/reset identity, validity, velocity, joint torque, contact flags, foot slip and heading. Captures reject warmup, command transitions, resets, failed steps, incomplete cycles and nonfinite arrays. Full cycles are phase-aligned, combined with a median and periodically smoothed. Optional symmetry is explicit (`--symmetry`) and should only be used for a gait with that symmetry. Every folded reference lists the exact source cycles and dispersion; bad/insufficient captures fail instead of falling back silently.

Per-command metrics include actual simulated forward speed, mean/RMS horizontal tracking errors, yaw error, torque², peak torque, contact-conditioned slip, heading drift, failures and timeouts. Failures during warmup still count. Metrics are simulator measurements, **not measured real-world speed**.

Promotion requires improvement in every high-speed segment with no additional failures, adequate samples, and bounded regressions in low-speed tracking, torque, slip and yaw. The comparator labels its decision provisional for that seed. Repeat with at least three held-out seeds and `--randomized`, and evaluate the Bumpy task. Freeze the best checkpoint/reference before any hardware comparison. Stop after two non-improving rounds; preserve failed rounds. Do not keep increasing speed/cadence merely to exhaust compute.

## Mystery / jump

The course Mystery task adds timed jump slots. `bootstrap jump TRACE OUT --env E --start A --end B` extracts a selected uninterrupted one-shot motion from a rollout, validates joint bounds and the scheduler's duration contract, and records provenance. It does not synthesize success from a trot or a reset. Inspect loading, flight, landing and recovery in simulation before using it. Mystery training requires an explicit reward JSON because omitting an appropriate landing/impact penalty is consequential; there is no claimed universal safe default. The course requires a TA's approval before this part's first hardware run.

## Validation and limitations

The NumPy capture tests and existing gait export/parity suite run locally. A real CPU MuJoCo-Warp two-command rollout with an explicitly zero policy verifies collector execution; it is only an infrastructure test. GPU training, promotion, randomized robustness and measured hardware speed are separate evidence gates. In restricted local environments set `MPLCONFIGDIR`, `XDG_CACHE_HOME` and `WARP_CACHE_PATH` to writable cache directories.

A concrete **simulation-only hypothesis** is provided in `mystery_experiment.json`: small bounded height/upward-motion incentives, always-on torso-clearance cost, joint acceleration/torque/action-change penalties, and retained tracking/upright objectives. These weights have not been validated as safe or optimal; reward penalties are not physical safety guarantees. Start from a demonstrated MixedGaits checkpoint, use a bounded pilot, examine all failures and landing loads, and only extend if it remains stable:

```sh
python -m mjlab.scripts.pupper_locomotion train --task Mjlab-Mystery-Flat-Pupper-v3 --checkpoint BEST_MIXED_CHECKPOINT --weights pupper-mjlab/configs/optional2/mystery_experiment.json --out artifacts/optional2/mystery-pilot --iterations 500
python -m mjlab.scripts.pupper_locomotion evaluate --task Mjlab-Mystery-Flat-Pupper-v3 --checkpoint PILOT_CHECKPOINT --out artifacts/optional2/mystery-eval
python -m mjlab.tasks.pupper_gait.bootstrap jump artifacts/optional2/mystery-eval/trace.npz artifacts/optional2/jump-reference --auto
```

`--auto` only selects an actual contact-free flight with sufficient height gain, loading and landing inside one valid uninterrupted episode. If none exists it fails explicitly. It does not label a fall or a reset as a jump. Combine a validated jump archive with the winning reach references in a new per-round directory before finetuning; preserve both manifests. The jump duration must fit the existing scheduler; a different timing requires modifying and testing that scheduler, not silently stretching the recorded motion.
