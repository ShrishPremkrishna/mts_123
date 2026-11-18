# Opt Lab 2 launch wrapper

`launch.py` wraps `python -m mjlab.scripts.pupper_locomotion train` so every run can be rebuilt
from `origin/markus/opt-lab-2` (REPRO_PLAN §4.1).

## Usage

From `pupper-mjlab/`:

```bash
uv run --no-sync python scripts/opt_lab_2/launch.py \
    --out ~/runs/opt-lab-2/gallop_c-s42 \
    --reference-dir configs/opt-lab-2/gallop_c \
    --weights configs/opt-lab-2/weights/splay.json \
    --iterations 3000 --seed 42
```

This trains into `~/runs/opt-lab-2/rp-gallop_c-s42/` as WandB run `rp-gallop_c-s42` (project `mjlab`).
It writes the log to `~/runs/opt-lab-2/rp-gallop_c-s42.log` and the launch record to `rp-gallop_c-s42.launch.json`.
`--name gallop_c-s42` is shorthand for `--out ~/runs/opt-lab-2/gallop_c-s42`. Set the root with `--runs-root`.

Warm start (Part 2 legs): add `--checkpoint ~/runs/opt-lab-2/rp-parent/model_2999.pt --iterations 500`.

## What it does

| Step | Behavior |
|---|---|
| Input check | Runs `git fetch origin markus/opt-lab-2`. Then, for every file under `--reference-dir` and for `--weights`, the file must be inside the repo, tracked, have no modified, staged or untracked changes (untracked files inside the reference dir also count), and the last commit that touched it must be an ancestor of `origin/markus/opt-lab-2`. If any check fails, it exits with code 2 before training starts. |
| Warnings (no refusal) | Warns if HEAD is not on origin, or if `pupper-mjlab/src` has uncommitted changes. Both are recorded. |
| Run name | Prefixes the `--out` basename with `rp-` unless it already has one or you pass `--no-prefix`. rsl_rl names the WandB run after this basename. |
| Physics defaults | Appends `--deployment-target-clamp --compose-com-randomization --locomotion-torso-contact --driver-limits <repo>/foundation/configs/class-driver-joint-limits.json` unless `--no-default-physics` is given. It does not add flags you passed yourself. |
| Pass-through | Passes every other flag through unchanged (`--iterations`, `--seed`, `--num-envs`, `--checkpoint`, `--profile`, `--logger`, `--task`, ...). It resolves relative `--checkpoint` and `--driver-limits` paths against your cwd. |
| WandB | Hooks `rsl_rl.utils.wandb_log_writer.WandbLogWriter.__init__` in the training process, without changing repo code. After `wandb.init` it adds the full launch record to `config.opt_lab_2`: HEAD SHA, origin SHA, `head_on_origin`, `src_dirty`, the reference dir and weights paths, per-file commit, git blob and sha256, the checkpoint and its sha256, and the train argv. It also writes a one-line summary to the run notes, adds the tag `rp` (plus `--tags`), and logs `reference-<dirname>` (all files in the dir) and `weights-<file>` as input artifacts (`use_artifact`). WandB deduplicates identical content into the same artifact version. |
| Logs | Tees stdout and stderr to `<out>.log` and writes `<out>.launch.json` next to the run dir. `launch.json` is also copied into the run dir. Pass `--quiet` to log to the file only. |

`--checked-bumpy` (Part 3) trains through `foundation/scripts/bumpy_checked_cli.py`: the reviewed 0.10 m Perlin grid plus an
accumulated physics-overflow audit (`<out>/physics_overflow_audit.json`). The stock 0.05 m grid overflows with the torso hull.
It requires `--task Mjlab-MixedGaits-Bumpy-Pupper-v3`, and its wrapper path and sha256 are recorded in the launch record.
Part 3 tooling (seam test, flat/Bumpy × DR eval, Bumpy fine-tune, export round trip, runbook `part3_run.py`) is described
in `report/part3_notebook.md` section 2.

With `--logger tensorboard`, every check and the log or launch record still happen, but nothing goes to WandB.
`--dry-run` runs the checks and prints the command and the launch record.

Without `--reference-dir`, no reference artifact is logged. The fast slot then falls back to the stock trot (see `report/part0_spelunking.md`).

## Notes

- Commit **and push** the reference dir and the weights JSON before launching. The wrapper refuses
  otherwise. Keep each reference round in its own directory (`npz` + `reference_manifest.json`).
- The wrapper never pushes anything and never uploads checkpoints beyond what the stock runner already uploads.
- Concurrency: see `report/infra/a100_throughput.md`. Use plain separate processes, never MPS.
