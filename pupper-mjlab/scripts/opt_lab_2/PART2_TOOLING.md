# Opt Lab 2 Part 2 tooling (W2)

Capture → fold → install → warm-start → evaluate, plus push test and share viewer. Run everything from
`pupper-mjlab/` with `export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl`. Standard physics flags (deployment target clamp,
driver limits, COM randomization, torso contact) are applied by default everywhere. Simulation only.

| Step | Code |
|---|---|
| capture, fold, verify | `src/mjlab/scripts/part2_reference.py` (tests `tests/test_part2_reference.py`) |
| suite, push, compare, annotate, batch | `src/mjlab/scripts/part2_eval.py` (tests `tests/test_part2_eval.py`; definitions `report/part2/eval/README.md`) |
| viewer | `scripts/opt_lab_2/viewer.py` |

Decisions that set defaults: `report/decisions/2026-11-18-w2-fold-default.md` (fold) and
`report/decisions/2026-11-18-w2-eval-metrics.md` (metrics, push protocol).

## Commands

```bash
P=~/runs/opt-lab-2/rp-<parent>/model_<N>.pt; R=configs/opt-lab-2/<parent-ref>   # omit --reference-dir for shipped refs

# 1. Capture: constant +1.5 and -1.5 m/s, 64 envs x 15 s each, first 3 s after each change/reset dropped,
#    heading hold off. Writes trace.npz (joint_pos, phase, contacts, validity...) + capture.json. Not committed.
uv run --no-sync python -m mjlab.scripts.part2_reference capture --checkpoint $P --reference-dir $R \
    --out ~/runs/opt-lab-2/capture-<name> [--commands 1.5,-1.5] [--seconds 15] [--num-envs 64]

# 2. Fold + install (decided default: lag 2.0 steps, median, 6 harmonics, pin all four leg_*_2 to the parent).
#    Writes captured_reach_gaits.npz + reference_manifest.json (loader contract), fold_summary.json,
#    diff_report.md, diff.png, then verifies the dir through the real loader in a fresh process.
uv run --no-sync python -m mjlab.scripts.part2_reference fold ~/runs/opt-lab-2/capture-<name> \
    configs/opt-lab-2/bootstrap/<name> [--variant naive|lag|phase|sym] [--lag-steps auto|none|X] \
    [--pin leg_back_r_2,leg_back_l_2|none] [--reduce median|mean|medoid] [--touchdown-fallback]
uv run --no-sync python -m mjlab.scripts.part2_reference verify configs/opt-lab-2/bootstrap/<name>
#    Read fold_summary.json: `warnings` (measured lag > 0.5 steps from 2.0) and `flagged_channels`
#    (near-constant parent channel moved) must be empty or justified before training.

# 3. Commit + push the reference dir, then warm-start (W0 launcher refuses unpushed inputs)
uv run --no-sync python scripts/opt_lab_2/launch.py --name <run> --reference-dir configs/opt-lab-2/bootstrap/<name> \
    --weights configs/opt-lab-2/weights/splay_stand_still.json --checkpoint $P --iterations 500 --seed 42

# 4. Evaluate with the reference the checkpoint trained on (suite = 16 schedules x 24 envs = 384 robot-runs)
uv run --no-sync python -m mjlab.scripts.part2_eval suite --checkpoint <ckpt> --reference-dir <ref> --name <run>
uv run --no-sync python -m mjlab.scripts.part2_eval push  --checkpoint <ckpt> --reference-dir <ref> --name <run>
uv run --no-sync python -m mjlab.scripts.part2_eval batch <ckpt>[:<ref>][=<name>] ... --modes suite push   # one GPU job at a time
uv run --no-sync python -m mjlab.scripts.part2_eval compare report/part2/eval/<a>.json ... --out report/part2/eval/<table>.md
#    compare refuses runs with different train num_envs or iterations past the parent (--allow-mismatch marks them).

# 5. Share viewer (prints a public viser URL; switch checkpoints in the GUI)
uv run --no-sync python scripts/opt_lab_2/viewer.py --checkpoint <ckpt> --reference-dir <ref> [--command 1.5,0,0]
```

Batch evaluations instead of running them next to training: an eval slows a co-scheduled trainer by ~27 %
(`report/decisions/2026-11-18-infra-num-envs.md`).

## Smoke validation (2026-11-18, A100 shared with W1 training)

Parent: `~/recovery/checkpoints/baseline-s42-cont4500/model_4498.pt` (shipped default references; fast slot = trot, 1× clock).

| Step | Result | Wall clock |
|---|---|---|
| capture (64 envs, 2 × 15 s) | +1.5: 891 clean cycles, vx 1.498, 1 fall; −1.5: 886 cycles, vx −0.149, 17 falls | 117 s (134 s incl. startup) |
| fold naive / lag(auto) / phase / sym | all verified by the loader; measured lag 2.18 / 1.94 steps; `phase` needed `--touchdown-fallback` forward (policy lands every ~8 steps, off the 37.5-step clock) | ~67 s each (two fresh-process loader imports dominate) |
| fold default (`smoke_cont4500_default`) | lag 2.0, measured 2.18 / 1.94, no warnings, no flagged channels, RMS diff vs parent 0.133 / 0.063 rad | 67 s |
| 20-iteration warm start via `launch.py` (1024 envs, `splay_stand_still.json`) on each of the 5 dirs | all exit 0; `model_4498` → `model_4517`; reference hashes in provenance | 98–115 s each |
| suite on the warm-started default checkpoint (`model_4517`, `--reference-dir .../smoke_cont4500_default`) | training block read from the run (1024 envs, s42, 20 iters past `model_4498`, ref sha matches); 61 falls (rev 23 / stop 30 / steady 8 / turn 0), err+1.5 0.193, vx +1.46 / −0.11, F run-max median 2.86 BW, p99 1.92 BW | 73 s |
| suite / push on the two baselines (`report/part2/eval/`) | see `summary.md` (unmatched: 3000 vs 1500 iterations past parent) | suite 63–73 s, push 114–143 s (512 envs) per checkpoint |

Per-variant numbers: `report/part2/fold_variants.md`. Smoke dirs: `configs/opt-lab-2/bootstrap/smoke_cont4500_*`
(smoke artifacts from a non-Part-1 parent; do not train Part 2 rounds on them).

## Known limitations

- The GPU rollout is not bit-reproducible at a fixed seed: identical suite runs of cont4500 gave 62–78 falls.
  Treat fall gaps under ~10–15 and single-sample peak forces as noise; rank on the decided statistics.
- Capture/eval sample at the 50 Hz control rate (foot-force peaks between samples are missed). The single-sample
  peak is a flag only: on the smoke checkpoint it read 5505 BW (one contact-solver spike) while the run-max median was 2.86.
- `phase` alignment assumes one touchdown of the chosen foot per reference cycle; it refuses otherwise.
- `sym` uses one mirror offset for both leg pairs (fine for trots, wrong for rotary gallops).
- The fixed 2-step lag relies on `mixed_gait_tracking` truncating `episode_length_buf - 1.8` to an integer; if
  that becomes a float, use `--lag-steps 1.8`.
