# Opt Lab 2 — Reproduction Plan (after losing the first GPU)

Branch: `markus/opt-lab-2` (the only remote branch we write to; never `main`).
Spec: https://cs123-stanford.readthedocs.io/en/latest/schedule/labs/fall-26/opt-lab-2.html

Tags: **[R]** recovered (seen in WandB configs/logs, surviving commit 59db016, or the old session's
reports) · **[I]** inferred from recovered evidence · **[N]** new / invented for this reproduction ·
**[H]** needs a human decision.

Evidence: `report/recovery/A_code_recovery.md` (diffs, weights), `B_repo_map.md` (code, CLI, constants),
`C_lineage_metrics.md` (run lineage, metrics, checkpoints). Checkpoints/metrics on the box: `~/recovery/`.

## 1. What survived and what didn't

| Item | Status |
|---|---|
| Task code, train/evaluate CLI, `bootstrap.py design/fold/compare`, reward terms | **Survives**: identical at 59db016 and `main` (`pupper-mjlab/src` unchanged) [R] |
| Final reward weights (`splay.json`, `splay_stand_still.json`) | **Recovered exactly** from WandB config.yaml [R] |
| PPO hyperparams, obs layout (48 × history 20 = 960), net 256-128-128-128 ELU | **Recovered** [R] |
| 23 checkpoints + ONNX (every Part 1/2 run) | **Recovered** to `~/recovery/checkpoints/` (129 MB) [R] |
| Per-iteration metrics of all 37 Nov-9/10 runs | **Recovered** to `~/recovery/metrics/` [R] |
| Fast-gait parameters (fast_*, gallop a/b/c) | **Lost** (only qualitative notes) |
| Reference `.npz` + manifests (gallop C, every bootstrapped round) | **Lost** (not in ONNX, not on WandB) |
| Capture/fold variants (naive, lag, lag_m2, phase, sym), abduction pin | **Lost** — must be rewritten |
| Extended eval (384 robot-runs, 3 Nm saturation, foot force, drift, splay), push test, viewers | **Lost** — must be rewritten |
| Report notebooks (part1/part2), reflection.md | **Lost** except fragments in `report/review/INDEX.md` |

Consequence: the recovered gallop/Part 2 checkpoints **cannot be cleanly warm-started**: the policy
observes the reference and that reference is gone. Baseline checkpoints (`baseline-s42-wandb`,
`baseline-s42-cont4500`) used the shipped references and remain fully valid, which makes them good for developing and testing tooling.

## 2. High-level approach (as taken the first time)

1. **Part 1 — fast gait sweep.** [R] Baseline (stock trot in fast slot) plus trot-family variants
   (`fast_trot`, `fast_stagger`, `fast_rotary`, `fast_wide`, `fast_long`) and then three rotary gallops
   (`gallop_a/b/c`). Each runs 3000 iterations, seed 42, 4096 envs, `--profile baseline` plus the physics flags below. A second seed (43)
   was run for the top candidates (~10 % seed spread).
2. **Pick on sim-to-real grounds, not reward.** [R] Gallop C was picked: lowest peak foot impact among
   gallops that ran backward (5.9 vs 6.7 BW), low heading drift (1.5 vs 7.8 deg/10 s), fewer reversal
   falls (15 vs 19 of 160), and at least two feet always down. Footfall LH → RH → RF → LF, and backward is the exact
   time reversal. Gallop A never learned backward (near-bound, drags front feet). C needs the most joint speed.
3. **Part 2 — bootstrapping on the chosen parent** [R]: capture at steady ±1.5 m/s → phase-align/fold
   → install `.npz` + manifest → warm-start 500-iteration legs with `splay_stand_still.json` weights.
   - The first round 1 **collapsed into crouching**: the captured reference shifted a near-constant joint
     channel (back-leg abduction) outside anything the policy had seen. **Fix: pin that channel to the
     parent's values.** [R]
   - Capture variants compared: `naive` vs `lag` (phase-shift by actuator/policy lag; the
     gait_tracking reward uses `phase_lead_steps 1.8`) vs `lag_m2` (the version used in all final rounds). [R names, I semantics]
4. **Iteration-matched evaluation** [R]: control = parent with its own reference and the same weights,
   continued; earlier rounds continued to the same budget; compare only at equal iterations past the parent
   (+500/+1000/+1500).
5. **Findings to reproduce or refute** [R]: bootstrapping consistently improves **tracking only**
   (0.159 → 0.124 at ±1.5 m/s). Most fall and jitter gains come from the stand-still reward and from longer
   training. Round 3 ≈ round 2-continued, a near tie, and round 2-continued was picked as final. Stand-still terms make standing
   push-recovery worse (6.7–10.4 % vs 5.4 % falls/push). Round 3 had a peak foot-force spike (6.1 BW).

## 3. Design principles recovered

- **Fixed iteration budgets; compare only at equal iterations; cut arms, not iterations.** [R, user rule]
- **Always run a control arm** with the same rewards and the same extra iterations, but no new reference. [R]
- **Rank on sim-to-real readouts** (falls, foot impact, 3 Nm saturation, heading drift, splay, push
  recovery), not training reward. Reward isn't comparable across reference dirs anyway. [R]
- **Treat one-seed gaps under ~10 falls / ~10 % as noise.** Report max-statistics (peak foot force) next to p99. [R]
- **Captured references are data:** sanitize before installing (pin near-constant channels, enforce
  periodicity, check joint limits). The loader rejects out-of-limit tables rather than clipping them. [R]
- **Report worse rounds as results.** Keep superseded designs in the notebook. [R, spec]
- **Hypothesis, comparator and metric for every change; keep sim and hardware evidence separate** (repo AGENTS.md). [R]

## 4. What makes this run easier (new) [N]

1. **Every run must be reproducible from the remote.** Commit **and push** reference dirs and weight JSONs
   *before* launching. Then log the reference dir (npz + manifest) and weights JSON to WandB as an
   artifact, and log the git SHA in the run config. Last time, references existed only in local worktrees.
2. **No NVIDIA MPS.** [R lesson] Twice, a GPU fault in one run killed every co-scheduled run. Run
   concurrent jobs as plain separate processes, and resume from the `save_interval=50` checkpoints.
3. **The A100 is fast, but it is one fixed pie** (measured 2026-11-18, `report/infra/a100_throughput.md`): 0.78 s/iter at 4096 envs solo
   (the old A10 ran 0.8–1.3 s/iter solo). Without MPS, **aggregate throughput is flat for 1–6 concurrent
   processes**, so concurrency does not add throughput. The 40-iteration benchmark said ≈ 1.27 it/s. **Full runs measured ≈ 1.0–1.1 it/s** (red-team review 2026-11-18, from run logs), so budget = total iterations / **1.1** s, and older estimates here are ~20–30 % low. One 3000-iteration run solo ≈ 45–50 min,
   Part 1 (~24 k iterations) ≈ 5 h, Part 2 (14 legs × 500 × 2 seeds) ≈ 3 h, plus evals. This supersedes the §5 estimate.
   Max concurrency **4** (`~/recovery/CONCURRENCY`, `report/decisions/2026-11-18-w0-concurrency.md`). Evals take a disproportionate
   share of the GPU, so batch them. Two seeds for every Part 2 arm is still affordable (≈ 3 h) and removes the one-seed caveat that blocked the final pick
   last time. (8192 envs gives 1.7× env-steps/s but changes PPO numerics: [H], not adopted.)
4. **Keep `--num-envs 4096`** so results stay comparable with the recovered numbers (PPO batch size). Use concurrency for throughput.
   Moving to 8192+ envs is a [H] decision.
5. **Build the extended eval harness and push test before Part 1 finishes**, against the valid baseline
   checkpoint, so every new checkpoint is scored the same way the moment it lands.
6. **Use the recovered metrics as targets/sanity checks** (e.g. gallop_c-s42-r2 reward ≈ 152, err_xy ≈ 0.25 at
   3000 iterations; matched Part 2 table in `C_lineage_metrics.md`).
7. **One reference per run, recorded in the manifest checksum.** Evaluate every checkpoint with the reference it trained on.
8. **Possible shortcut (untested idea) [N]:** recover a self-consistent reference for an old checkpoint by fixed-point iteration. Roll out the
   frozen policy with a guessed reference, capture, fold, reinstall, repeat. Only useful for comparing against the old
   policies. Low priority.

## 5. Work plan

Agents work in their own git worktrees (`~/wt/<name>`). Each commits only its own files, rebases onto
`origin/markus/opt-lab-2` and pushes `HEAD:markus/opt-lab-2`. WandB project `mjlab`, run names prefixed `rp-`.

| # | Workstream | Depends on | Tag |
|---|---|---|---|
| W0 | Infra: A100 concurrency benchmark (1–6 jobs, no MPS), `launch` wrapper (pushed-SHA check, WandB artifact of ref + weights), Part 0 spelunking report | – | [N] infra, [R] report contents |
| W1 | Part 1: re-author baseline + `fast_trot` + gallop a/b/c with `bootstrap.py design`, check in `visualize_reference`, train 3000 iterations at s42 (plus s43 for the top 2), evaluate, notebook. Gallop C *criteria* recovered, *parameters* re-invented | W0 launch wrapper (soft) | [R] recipe, [I] params |
| W2 | Part 2 tooling: capture + fold (naive/lag/phase, periodicity, abduction pin, symmetrize), extended eval (falls over scripted command changes, ±1.5 m/s tracking, 3 Nm saturation, peak/p99 foot force, heading drift, splay, standing jitter, slip, top speed), push test (0.5/1.0 m/s), viser share viewer | – (dev on baseline ckpt) | [N] code, [R] metric definitions |
| W3 | Part 2 rounds: r1 (naive vs lag), r2, r3 in 500-iteration legs. Control and continuations to +1500 at **s42 and s43**. Matched table, reward-term trajectories, notebook | W1 parent + W2 | [R] design, [N] 2nd seed |
| W4 | Part 3 (sim): full MixedGaits sweep 0 → threshold → back, seam stumbles, Bumpy + Lab 4 DR pass, export `policy.json`. Hardware is human-only | W3 final | [R] spec |
| – | Part 4 (Mystery task): not started. Hardware needs TA approval | – | [H] |

**Human decision points [H] are made lazily** ([decisions/README.md](decisions/README.md)): a fresh grader agent
decides from the evidence and writes a compact decision doc, work continues on that decision, and the human reviews
afterwards. This covers the Part 1 parent gait, the final Part 2 round, any num_envs/budget changes, and the Part 4 sim scope.
Physical actions (hardware deployment, Part 4 hardware) still need a human and TA approval. A grader can only
decide how to *prepare* them.

**Compute budget [N, estimate]:** Part 1 at ≈ 6 runs × 3000 iterations, run concurrently, takes about 1.5–2 h. Part 2 at ≈ 14 legs × 500 iterations × 2 seeds
takes about 2–3 h. Evals add roughly 30 min.
