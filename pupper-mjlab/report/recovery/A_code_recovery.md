# A. Code recovery from W&B per-run files (opt-lab-2, MixedGaits, fast gait + bootstrapping)

Legend: **[R]** recovered (seen verbatim in a downloaded file or the surviving repo); **[I]** inferred (reasoned from indirect evidence, not seen).

Files:
- Raw downloads: `~/recovery/wandb_files/<run>/{mts_123.diff,config.yaml,requirements.txt,wandb-metadata.json[,output.log]}` (37 runs created on 2026-11-09 or later; `p2g-r3/onnx/p2g-r3.onnx` as a probe).
- Extracted `git diff` part of every run: `~/recovery/per_run_diffs/<run>.patch`.
- Scripts: `/tmp/claude-1000/-home-ubuntu/1e6973a7-fdc0-4a4d-af73-a538d53a1bf0/scratchpad/scripts/`.
- Branch `recovered` (local only, not pushed) in `/tmp/claude-1000/-home-ubuntu/1e6973a7-fdc0-4a4d-af73-a538d53a1bf0/scratchpad/src/mts_123`, checked out at `/tmp/claude-1000/-home-ubuntu/1e6973a7-fdc0-4a4d-af73-a538d53a1bf0/scratchpad/wt_recovered`.

Missing downloads: `config.yaml` and `output.log` do not exist on W&B for the crashed runs `fast_rotary-s42` and `p2-r1-{control,naive,jitter,phase,sym,sym-protect}`. Only the diff and requirements files were uploaded for those runs. **[R]**

---

## 1. What the diffs contain (the main finding)

**The W&B diffs contain no source code.** Across all 37 runs the only tracked-file changes are uncommitted edits to `pupper-mjlab/report/review/INDEX.md` (20 runs) and `pupper-mjlab/report/part1_notebook.md` (3 runs). Seventeen runs have an empty `git diff`. **[R]**

The diffs also come from a different checkout than the code that ran:
- `wandb-metadata.json` `root` (cwd) and `git.commit` point to a **worktree**: `/home/ubuntu/mts_123/.claude/worktrees/<name>/pupper-mjlab` at the run's commit (exec-merge=355b46a, exec-round2=42a4f01, exec-p2r1=27808ec, gallop-dd213ab, p2-ba6a5f9, p2b-c1f269c, p2r2-dc21aeb, p2r3-7aa76f8). **[R]**
- The `p2g-r1-bootstrap` traceback shows the imported code ran from that worktree's `src/` (for example `.../worktrees/p2-ba6a5f9/pupper-mjlab/src/mjlab/scripts/pupper_locomotion.py`). **[R]**
- The `mts_123.diff` header (`git commit`, `git status`, `git diff`) describes the **main checkout** `/home/ubuntu/mts_123` (branch `opt-lab-2`). Its untracked list contains `.claude/`, which is where the worktrees live, and the diff is written next to `--out` (which is under the main checkout). **[R]** The likely cause is that rsl_rl's `store_code_state` resolved the repo from the log directory. **[I]**
- So each diff is `git diff` of the main checkout against **its own HEAD** at run start. It is not cumulative, it is not against 59db016, and it says nothing about the worktree the code ran from (any dirty state there was never captured). **The content of every lost commit is absent.** **[R]**

### Commits seen (diff-header HEAD vs. the worktree commit W&B recorded)

| Time (UTC, 11-09) | Main-checkout HEAD (diff header) | Code commit (wandb git / cwd) | Runs |
|---|---|---|---|
| 13:42 | 59db016 | 59db016 (main checkout itself) | baseline-s42-wandb |
| 14:20 | 59db016 | **355b46a** exec-merge | fast_trot/stagger/rotary-s42 |
| 15:00 | 42a4f01 | **42a4f01** exec-round2 | fast_wide-s42, fast_long-s42 |
| 15:49–16:32 | **28a4d1c** (lost; not in your list) | 42a4f01 | baseline-s42-cont4500, baseline-s43, fast_long-s43 |
| 18:20 | **3219d63** (lost; not in list) | 42a4f01 | p2-r1-control |
| 18:33 | 27808ec | **27808ec** exec-p2r1 | p2-r1-{naive,phase,sym,sym-protect,jitter} |
| 18:46–19:53 | dd213ab | **dd213ab** | gallop_{a,b,c}-s42 (+ -r2) |
| 20:27 | ba6a5f9 | **ba6a5f9** | p2g-r1-{naive,bootstrap,jitter} |
| 20:35 | **b916f17** (lost; not in list) | ba6a5f9 | p2g-r1-*-r2 |
| 21:37 | **32c2210** (lost; not in list) | ba6a5f9 | p2g-ctrl-check |
| 21:39 | c1f269c | **c1f269c** | p2g-r1b-{bootstrap,jitter} |
| 22:08–22:26 | dc21aeb | **dc21aeb** | p2g-r2, p2g-control1000 |
| 22:28 | 7aa76f8 | **7aa76f8** | p2g-r3 |
| 11-10 01:36–02:25 | **8492bb6** (lost; not in list) | dc21aeb / c1f269c | p2g-r2cont, p2g-ctrl1500, p2g-r1cont-a/b |

None of these commits except 59db016 exist in the clone, and neither do the diff preimage blobs (2b58838, 0984844, 91cfdbf). **[R]**

### `git apply --check` onto 59db016

- **Empty, so trivially applicable (17):** baseline-s42-wandb, baseline-s42-cont4500, baseline-s43, fast_long-s42/s43, fast_trot/stagger/rotary/wide-s42, gallop_{a,b,c}-s42, p2-r1-{naive,jitter,phase,sym,sym-protect}.
- **Fail (20):** every other run, with `pupper-mjlab/report/review/INDEX.md: No such file or directory` (and the same for `report/part1_notebook.md` in the three `*-r2` p2g runs). At 59db016 `pupper-mjlab/report/` does not exist; the main checkout showed it as untracked at 14:20, and it was tracked by 42a4f01. **[R]**
- **No diff applies cleanly with real content.** **[R]**

### What each diff changes

| Diff (md5 group) | Runs | Base | Files | Summary |
|---|---|---|---|---|
| 37252600 | baseline-s42-wandb | 59db016 | none | clean tree |
| 0cd6fc14 | fast_trot/stagger/rotary | 59db016 | none | `report/` untracked |
| 3f4dd434 | fast_wide, fast_long-s42 | 42a4f01 | none | report/media, report/review/references untracked |
| e959fae8 / 338ef11c | cont4500 / s43 runs | 28a4d1c | none | report/data/commands_sweep.json, media/policy_* untracked |
| a184ef64 | p2-r1-control | 3219d63 | INDEX.md | adds the "Human review of fast_long" table (jitter when standing, desperate at 2–2.5 m/s, reversal falls), notes on the To-review rows, and the "Measured follow-ups" heading (`rev-`, `over-` evals) |
| 01a44d15 | p2-r1-* | 27808ec | none | clean apart from `.claude/` |
| 4cd018ac | gallop_*-s42 | dd213ab | none | report/reflection.md, review/STATUS.md untracked |
| 9171fdd1 | gallop_*-r2 | dd213ab | INDEX.md | +1 row: gallop references footfall LB, BR, FR, FL; backward is the exact time reversal; `a` tips forward and drags the left hind foot; `c` needs the most joint speed. `weights/splay_stand_still.json` is **untracked** |
| a8b45cfd | p2g-r1-* | ba6a5f9 | INDEX.md | same row; `bootstrap/p2r1_gallop_b_{lag,naive}/` untracked |
| 7bac4dd7 | p2g-r1-*-r2 | b916f17 | INDEX.md, part1_notebook.md | "The pick": gallop c chosen over b (lands at 5.9 vs 6.7 body weights; forward drift 1.5 vs 7.8 deg per 10 s; reversal falls 15 vs 19 of 160; weakness: hind splay and backward roll −0.14 rad / 5.1°); "Part 2 round 1 references were captured from c (commit ba6a5f9)" |
| f05101100 / e864156f / 5c0f1ea0 / 014c2801 / 13068ed9 / 1f2fad65 / 565c1656 | ctrl-check … r1cont-b | 32c2210 … 8492bb6 | INDEX.md | same single row, never committed. Untracked lists grow: data/evals/{full,sweep,video}-gallop_*, tables/gallop_continuous_sweep.md, gallop_highspeed_3seeds.md, part2_notebook.md (22:08), commands_footfall.json and evals/{foot-*,quick-hs-r2-final-e101,quick-rev-r2-final-e101} (22:26), **commands_push.json (02:25)** |

---

## 2. What changed in code: evidence beyond the diffs

### Training CLI
- Every flag the runs used already exists in 59db016's `src/mjlab/scripts/pupper_locomotion.py`: `--reference-dir`, `--profile {baseline,fast}`, `--weights`, `--deployment-target-clamp`, `--compose-com-randomization`, `--locomotion-torso-contact`, `--driver-limits`, `--checkpoint`, `--iterations`, `--num-envs`, `--seed`, `--logger`. **[R]**
- `--reference-dir` sets `PUPPER_REFERENCE_DIR` before task registration. `--weights` JSON replaces all weights: unknown names are rejected and unnamed terms are set to 0. `--checkpoint` drops `hold_base_airborne` and `pretrain_rewards`, and the config.yaml files confirm this. **[R]**
- **`pupper_locomotion.py` did change by ba6a5f9.** Traceback line numbers are `runner.learn` at 166 (161 at 59db016), `else: train(args)` at 374 (358) and `__main__` at 376 (360). That means 5 lines were added at or before `train()`'s learn call and about 11 later (evaluate or argparse). The content is unknown. **[R line numbers / I meaning]**
- The other traceback frames (`tasks/pupper/rl/runner.py:83`, `rl/vecenv_wrapper.py:77`, `envs/manager_based_rl_env.py:440`, `managers/reward_manager.py:125`, `tasks/velocity/mdp/rewards.py:94`) match 59db016 line for line. Those files most likely did not change. **[R/I]**
- The resolved `env_cfg` is **byte-identical across all runs** apart from reward weights, the checkpoint-driven warmup removal, run name and output path. Observations, actions, events, commands, terminations, gait params and reward params all match. So the lost commits did not change the task config that training sees. The differences between runs come from reference-dir data and weight JSONs. **[R]**
- `requirements.txt` is identical in all 37 runs: mjlab 1.5.2, rsl-rl-lib 5.4.0, torch 2.9.0+cu128, mujoco 3.10.0, mujoco-warp 3.10.0.2, warp-lang 1.14.0, wandb 0.22.3. **[R]**

### Reward terms and weights per run (from config.yaml) [R]

All runs share: track_linear_velocity 10, track_yaw_velocity 7, gait_tracking 10, upright 1, lin_vel_z_l2 −0.5, ang_vel_xy_l2 −0.05, action_rate_l2 −0.02, knee_ground_contact −1. These are the 59db016 `BASE_WEIGHTS`, which the `baseline` profile uses.

| Weights file | Extra nonzero terms | Runs |
|---|---|---|
| (none, `--profile baseline`) | — | baseline*, fast_*, p2-r1-{control,naive,phase,sym} (no p2g run used the bare profile) |
| `weights/stand_still.json` | stand_still_joint_velocity, stand_still_pose (values unknown: crashed run, no config) | p2-r1-jitter (ref = fast_long) |
| `weights/protect_symmetry.json` | roll_asymmetry (value unknown; logged reward −0.036 vs −0.0075 at w=−1 in gallop runs, so the weight was possibly larger) | p2-r1-sym-protect |
| `weights/splay.json` | abduction_angle **−0.1**, roll_asymmetry **−1** | gallop_*, p2g-r1-{naive,bootstrap}(+r2), p2g-ctrl-check, p2g-r1b-bootstrap |
| `weights/splay_stand_still.json` | splay + stand_still_joint_velocity **−0.1**, stand_still_pose **−0.5** | p2g-r1-jitter(+r2), p2g-r1b-jitter, p2g-r2, p2g-r2cont, p2g-r3, p2g-control1000, p2g-ctrl1500, p2g-r1cont-a/b |

All of these terms exist at 59db016, and `baseline-s42-wandb` at clean 59db016 lists them with weight 0. "Splay" and "stand-still" were weight files, not new reward code. **[R]**

### Full reward list for the final runs (p2g-r3, p2g-r2cont, p2g-control1000, p2g-ctrl1500, identical) [R]

| Term | Weight | Function | Params |
|---|---|---|---|
| track_linear_velocity | 10 | velocity.mdp.rewards.track_linear_velocity | std 0.3162 (√0.1) |
| track_yaw_velocity | 7 | pupper.mdp.rewards.track_yaw_velocity | sigma 0.25 |
| gait_tracking | 10 | pupper_gait.mdp.mixed_gaits.mixed_gait_tracking | frequency 1.3333, n_samples 100, blend_speed 0.1, gallop_speed 0.5, phase_lead_steps 1.8, std {.\*_1: 0.15, .\*_2: 0.3, .\*_3: 0.3} |
| upright | 1 | velocity.mdp.rewards.upright | base_link, std 0.4472 |
| lin_vel_z_l2 | −0.5 | pupper.mdp.rewards.lin_vel_z_l2 | |
| ang_vel_xy_l2 | −0.05 | pupper.mdp.rewards.ang_vel_xy_l2 | |
| action_rate_l2 | −0.02 | envs.mdp.rewards.action_rate_l2 | |
| knee_ground_contact | −1 | velocity.mdp.rewards.self_collision_cost | sensor knee_ground_contact, force_threshold 1 |
| abduction_angle ("splay") | −0.1 | pupper.mdp.rewards.joint_deviation_l2 | joints `leg_.*_2` |
| roll_asymmetry | −1 | pupper_gait.mdp.mixed_gaits.roll_asymmetry | tau_s 0.3 |
| stand_still_joint_velocity | −0.1 | pupper.mdp.rewards.stand_still_joint_velocity | joints .\*, command_threshold 0.05 |
| stand_still_pose | −0.5 | pupper.mdp.rewards.stand_still_pose | joints .\*, command_threshold 0.1 |
| air_time, base_height (target 0.145, std 0.05, gallop_std 10), foot_slip, joint_acc_l2, joint_torques_l2, orientation_l2, self_collision_l/r, termination, track_angular_velocity | 0 | | |

### PPO hyperparameters (final runs) [R]

- Runner: OnPolicyRunner; experiment `pupper_foundation_optional2`; num_steps_per_env 24; save_interval 50; seed 42; 4096 envs; 500 iterations.
- Lineage: p2g-r3 started from p2g-r2/model_3996 and ended at 4495. p2g-r2cont started from the same checkpoint.
- Actor and critic: MLP [256, 128, 128, 128] with ELU and obs normalization; scalar std, init 1.0.
- PPO: clip 0.2, desired_kl 0.01, adaptive schedule, lr 1e-3, entropy 0.01, gamma 0.97, lambda 0.95, 5 epochs, 4 mini-batches, max_grad_norm 1, value_loss_coef 1, clipped value loss; no symmetry or RND.
- Env: decimation 5, episode 10 s; action scale 0.75 with clip, driver-limit ranges baked in.
- Commands: vx ±1.5, vy ±0.5, wz ±2; 2% standing envs; 15% forward, 15% turn.

### Reference loading (fast_* and bootstrap/* dirs) [R from 59db016 code; I that the lost dirs used it]

- `gait_reference.py` reads `$PUPPER_REFERENCE_DIR/captured_reach_gaits.npz` (keys `reach_fwd`, `reach_back`) and `reference_manifest.json`. The manifest has `frequency_mult`, `reference_sha256` (checked) and per-direction checks.
- `frequency_mult` sets `_GAIT_FREQ_MULT["reach"]` and `["reach_back"]`. The fast slot plays at |vx| ≥ gallop_speed 0.5 with frequency 1.333 × mult.
- Backward captures are stored in forward chronological order and reversed once by the loader.
- The 59db016 example `configs/optional2/extended_trot/` (half_stride 0.08, stance_z −0.12, swing_lift 0.05, 5 swing keyframes, ×2 clock) is the template. **[R]**
- The lost `configs/opt-lab-2/{fast_*,bootstrap/*,weights/*}` were **tracked in the lost commits**. `git status` lists only individual new entries, for example `weights/splay_stand_still.json` and `bootstrap/p2r1_gallop_b_{lag,naive}/`, never the parent directories, so those parents already held tracked files. **[I, strong]**

### Gait parameters per fast gait
**Not recoverable from W&B.** They lived in the `.npz` and manifest of each reference dir. Known facts:
- fast_trot and fast_stagger: modest swing clearance. **[R, INDEX.md]**
- fast_wide backward: a front leg is thrown high and outward. **[R]**
- fast gaits put about 11% of torque samples at the 3 Nm limit. **[R]**
- gallop_a/b/c footfall: forward LB→BR→FR→FL; backward is the exact time reversal. **[R]**
- gallop_a tips over open loop and drags the left hind foot; gallop_c needs the most joint speed. **[R]**
- gallop_a can't run backward; c was picked for Part 2. **[R, part1_notebook hunk]**
- No stride, stance_z, swing_lift or frequency_mult numbers exist for any fast_* gait.

### Capture/fold pipeline variants
- 59db016 `bootstrap.py` already has `fold` (phase-aligned median of full cycles at ±speed, periodic smoothing with 6 harmonics, `--symmetry` half-period mirror average, `--frequency-mult`), `design`, `compare` and `jump`. **[R]**
- Variant dirs used:
  - Round 1, on fast_long: `bootstrap/r1_naive`, `r1_phase`, `r1_sym`. **[R]** These map to the plain fold, a phase variant (unknown) and `--symmetry`. **[I]**
  - Gallop round 1, on gallop_c-s42-r2/model_2998: `p2r1_gallop_c_naive`, `p2r1_gallop_c_lag`, plus `p2r1_gallop_b_{naive,lag}` (untracked, never trained). **[R]**
  - Round 1b: `p2r1b_gallop_c_lag_m2`; round 2: `p2r2b_lag_m2`; round 3: `p2r3_lag_m2`. **[R]**
- "lag" probably shifts the captured phase by the policy or actuator delay (gait_tracking uses phase_lead_steps 1.8), and "m2" is a modified second version. **[I, weak]**
- **The lag, lag_m2 and phase implementations, and the "pin back-leg abduction to the parent" step, appear in no W&B file. Not recoverable.** The only related signal is the 5+11-line growth of `pupper_locomotion.py` by ba6a5f9 (content unknown).

### Eval / push-test scripts
- 59db016 `pupper_locomotion evaluate` (commands JSON, trace.npz, metrics) is present. **[R]**
- Later evals are known only by name: `report/data/commands_{sweep,capture,overspeed,reversal,footfall,push}.json` and `data/evals/{full-gallop_*-e101/202/303, sweep-*, video-*, foot-*, quick-hs-*, quick-rev-*, rev-*, over-*}`. **[R names]**
- 59db016 has no push mechanism ("Push events are disabled"), so the push test (commands_push.json, 11-10 02:25) needed code or a script that is lost. **[I]**

---

## 3. Branch `recovered` (local, from 59db016)

| Commit | Content | Status |
|---|---|---|
| 7f21d5f | `configs/opt-lab-2/weights/splay.json`, `splay_stand_still.json` | Exact effective weights **[R]** |
| 7a2ea07 | `weights/stand_still.inferred.json`, `protect_symmetry.inferred.json` | Term set **[R]**, magnitudes **[I]** |
| 6479a4e | `report/review/INDEX.md` (rebuilt from hunks, gaps marked), `report/part1_notebook.md` ("The pick" fragment only) | **[R]** text, partial |

No code diff could be applied, because none exists.

## 4. Not recoverable (from W&B)

- **All source code in the lost commits:**
  - The pupper_locomotion.py additions (about 16 lines).
  - Any bootstrap/fold changes: lag, lag_m2, phase, pinning.
  - Eval, push-test and footfall scripts.
- **All reference data:** `configs/opt-lab-2/fast_{trot,stagger,rotary,wide,long,gallop_a,gallop_b,gallop_c}/` and `bootstrap/{r1_*,p2r1_*,p2r1b_*,p2r2b_*,p2r3_*}/` (`.npz` files and manifests). The gait parameters and frequency multipliers go with them. The ONNX metadata carries no reference tables, and the deploy `policy.json` that the logs say was uploaded is not on W&B now.
- **Exact `stand_still.json` and `protect_symmetry.json` values**, and whether any weight JSON listed zero-weight terms.
- **The report tree:** part1/part2 notebooks, part1_gallop_design.md, reflection.md, STATUS.md, tables, data/evals, media, review/* verdicts. Only the INDEX.md and part1_notebook.md fragments survive.
- **Commit messages and authorship** of 355b46a … 8492bb6, including the extra main-checkout commits 28a4d1c, 3219d63, b916f17, 32c2210 and 8492bb6.

Still on W&B and worth fetching if needed: `model_*.pt` checkpoints and `<run>.onnx` for every run (for example p2g-r3/model_4495.pt and p2g-r2cont/model_4495.pt).
