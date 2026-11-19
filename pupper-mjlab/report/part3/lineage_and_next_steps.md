# Training lineage, rollback, and the "fusion" experiment: costs for a decision

2026-11-19. Simulation only. This doc reads existing files, run logs, checkpoints and WandB. It trains nothing.
Throughput used throughout: **~1.07 it/s for the whole A100 at 4096 envs** (flat terrain), whatever the number of processes.

## Answers in brief

| Question | Answer |
|---|---|
| 1. Do we retrain from scratch or continue? | **We continue.** Only Part 1 trains from scratch (3000 iterations). Every Part 2 and Part 3 run is a **warm start**: it loads the previous checkpoint and adds 500 iterations. The shipped Part 3 candidate is 6500 PPO updates from random weights: 3000 + 1500 + 2000. |
| 2. Roll back Part 3 Bumpy to a safe point? | **Branch from the Part 2 final P (`model_4496`).** Nothing after P on the Bumpy line is push-safe. A reward change applies cleanly by warm start, and the critic re-fits within about 5–10 iterations. A 45-minute run fits **one arm × +2000** (or 2 × +1000, or 4 × +500). The GPU does not run more work in parallel. Branching costs about 0.6 GPU-h per arm. Retraining from scratch costs about 1.7 GPU-h per arm, plus two capture rounds. |
| 3. Fusion: rerun all Parts? | **Strictly, yes.** A new clock means a new reference. That retrains Part 1, re-captures Part 2, and reruns Part 3: about **4.3 GPU-h of training** and **5.5–6 h wall** with critical-path scheduling (about 8 h without it). **The cheaper variant** re-clocks the Part 2 reference and warm-starts from P: about **0.6–1.2 GPU-h** and **1–1.5 h**, labelled as a deviation. Do the push fix and reward-shaping sweep first either way. |

## Terms

| Term | Meaning |
|---|---|
| Iteration | One PPO update: 24 env steps × 4096 envs = 98,304 samples, then 5 epochs × 4 mini-batches. |
| From scratch | Random actor and critic weights. Includes the 500-iteration "held aloft" pretrain phase (`hold_base_airborne` event and `pretrain_rewards` curriculum). |
| Warm start | `--checkpoint <model_N.pt>`: the new run loads that checkpoint's actor, critic and optimizer state, then trains. The pretrain phase is removed (`pupper_locomotion.py:140–143`). |
| Leg | One warm-started training run (500 iterations in Parts 2 and 3). |
| +N | N iterations past the run's warm-start parent. Part 2 counts from the Part 1 parent; Part 3 counts from P. |
| Index | The number in `model_<index>.pt`. A warm start resumes at the parent's last index, so 500 updates advance the index by 499. That gives 2999 → 3498 → 3997 → 4496 (Part 2) → 4995 → 5494 → 5993 → 6492 (Part 3). |
| Reference (dir) | The gait table the policy **observes** and is rewarded for tracking (`captured_reach_gaits.npz` + `reference_manifest.json`). A checkpoint is only valid with the reference it trained on. |
| Clock | `frequency_mult` in the manifest. The stride period is 0.75 s / mult. gallop C uses **2.25× = 3.0 strides/s**; 2.5× would be 3.33 strides/s. |
| Capture / fold | Part 2 bootstrapping. Roll out a policy at ±1.5 m/s, record joint angles (capture), and fold them onto one stride by phase (fold). This gives the next round's reference. |
| P | Part 2 final: `rp-p2-s42-r2-cont-1500/model_4496.pt`, the warm start of Part 3. |
| flat_dr / bumpy_dr | The 384-run eval suite on flat or Bumpy ground with Lab 4 domain randomization (DR). "Falls" = tilt > 30°. |
| nompd | The same suite at nominal motor gains (kp 5.5, kd 0.25), with the PD-gain randomization removed. |
| push_stand@0.65 | Share of 0.65 m/s sideways shoves that topple a standing robot (flat_dr). **v50** = the shove speed that topples half. |
| HOLD (a)/(b)/(c) | Hardware gates. (a) nompd − flat_dr ≤ 15. (b) nompd ≤ nompd(P) + 10 = 48. (c) push_stand@0.65 ≤ P + 0.10 = 0.209. Each is a two-seed mean. |
| WIDEN | The Part 3 training PD-gain range kp ×[0.6, 1.3], kd ×[0.7, 1.5]. Evaluation stays on Lab 4's kp ×[0.6, 1.1], kd ×[0.8, 1.5]. |
| GPU-h | Hours of the whole GPU at 1.07 it/s: **1000 flat iterations = 0.26 GPU-h (15.6 min)**. Bumpy costs about 1.15× per iteration, so 1000 Bumpy iterations ≈ 0.30 GPU-h. |
| Critical-path scheduling | Run the dependency chain that gates the next decision first. Speculative and report-only arms fill the gaps. Hand off downstream the moment a decision lands ("metapiping", `infra/wallclock_retro.md`). |

---

## Q1. Training paradigm and lineage

### 1.1 The tree (seed-42 line; ★ = selected)

Runs live under `~/runs/opt-lab-2/`; "rp-" is omitted below. Wall-clock is launch → last checkpoint and includes sharing the GPU with *c* concurrent trainers.

```
[scratch] p1-gallop_c-s42  1c5r0n8c  Flat  ref gallop_c@e01364f (2.25×)  splay.json        it 0→2999   2.46 h (c=3)
│   (the Part 1 parent; every s42 arm below warm-starts from model_2999)
│
├─ capture p0 → fold → refs p2/s42/r1-{lag,naive,backpin} @1611a75
│
├── p2-s42-ctrl-500   fqt1qabi  Flat  ref gallop_c (parent's own)   splay_stand_still  2999→3498  0.51 h
│   └── ctrl-1000     62culz7u                                                          3498→3997  0.53 h
│       └── ctrl-1500 eoohixa0                                                          3997→4496  0.50 h   (matched control)
├── p2-s42-r1-naive   xoit2y5d  Flat  ref p2/s42/r1-naive           splay_stand_still  2999→3498  0.51 h   (dead end)
├── p2-s42-r1-backpin fx4oyomc  Flat  ref p2/s42/r1-backpin         splay_stand_still  2999→3498  0.51 h   (dead end)
└── p2-s42-r1-lag     t68nrwkw  Flat  ref p2/s42/r1-lag             splay_stand_still  2999→3498  0.50 h   (+500)
    ├── r1-cont-1000  jkh7iqck  Flat  ref p2/s42/r1-lag                                3498→3997  0.45 h
    │   └── r1-cont-1500 e79oihhl                                                      3997→4496  0.50 h
    ├─ capture r1-lag/model_3498 → fold → ref p2/s42/r2-lag @60ce5a4   (≈5 min)
    └── p2-s42-r2     2i9g5098  Flat  ref p2/s42/r2-lag             splay_stand_still  3498→3997  0.51 h   (+1000)
        ├─ capture r2/model_3997 → fold → ref p2/s42/r3-lag @90b3e59
        ├── p2-s42-r3 rrdkrrzw  Flat  ref p2/s42/r3-lag                                3997→4496  0.52 h   (runner-up)
        └── ★ p2-s42-r2-cont-1500 wmhcp24q Flat ref p2/s42/r2-lag  splay_stand_still  3997→4496  0.50 h   = P (+1500)
            │   (every Part 3 run uses ref p2/s42/r2-lag, splay_stand_still, WIDEN PD ranges, seed 42)
            ├── p3-final-bumpyft-kpwide-smoke   5rs6yzsh  Bumpy  4496→4545 (50 it)   0.04 h
            ├── p3-final-flatctrl-kpwide-smoke  s3qcq7j0  Flat   4496→4545 (50 it)   0.04 h
            ├── Bumpy arm (checked Bumpy, Lab 4 DR + WIDEN)
            │   leg1 bhj73f64 4496→4995 (+500)  0.32 h
            │   └ leg2 0mnrsqsi 4995→5494 (+1000) 0.32 h
            │     └ leg3 4spb61ib 5494→5993 (+1500) 0.36 h
            │       └ ★ leg4 23pt1499 5993→6492 (+2000) 0.35 h   = sim candidate, HOLD(c) (push)
            └── Flat control arm (same everything, Flat task)
                leg1 jtz42coz 4496→4995 (+500)  0.32 h
                └ leg2 aagiyf2k 4995→5494 (+1000) 0.31 h
                  └ leg3 clhpdudm 5494→5993 (+1500) 0.35 h
                    └ leg4 13m4vcia 5993→6492 (+2000) 0.34 h   = clears all four HOLD rules
```

The Part 3 dry-run smokes from the Part 1 parent are off the line: `p3-parent-bumpyft-smoke` c3rfre1p, `-flatctrl-smoke` rsrp5o5i, and the two kpwide smokes d2588xjf / c90rcnpd.

### 1.2 The same, as a table (s42 line)

Task "Flat" = `Mjlab-MixedGaits-Flat-Pupper-v3` (the CLI default); "Bumpy" = `Mjlab-MixedGaits-Bumpy-Pupper-v3` through the checked-Bumpy wrapper. Weights live in `configs/opt-lab-2/weights/`. Reference dirs live in `configs/opt-lab-2/`.

| Run | WandB | Task | Reference dir | Weights | Warm-start parent | Index range | Wall h (c) |
|---|---|---|---|---|---|---|---|
| p1-gallop_c-s42 | 1c5r0n8c | Flat | gallop_c | splay | scratch | 0→2999 | 2.46 (3) |
| p2-s42-ctrl-500 | fqt1qabi | Flat | gallop_c | splay_stand_still | p1-gallop_c-s42/model_2999 | 2999→3498 | 0.51 (4) |
| p2-s42-ctrl-1000 | 62culz7u | Flat | gallop_c | splay_stand_still | ctrl-500/model_3498 | 3498→3997 | 0.53 (4) |
| p2-s42-ctrl-1500 | eoohixa0 | Flat | gallop_c | splay_stand_still | ctrl-1000/model_3997 | 3997→4496 | 0.50 (4) |
| p2-s42-r1-naive | xoit2y5d | Flat | p2/s42/r1-naive | splay_stand_still | p1/model_2999 | 2999→3498 | 0.51 (4) |
| p2-s42-r1-backpin | fx4oyomc | Flat | p2/s42/r1-backpin | splay_stand_still | p1/model_2999 | 2999→3498 | 0.51 (4) |
| p2-s42-r1-lag | t68nrwkw | Flat | p2/s42/r1-lag | splay_stand_still | p1/model_2999 | 2999→3498 | 0.50 (4) |
| p2-s42-r1-cont-1000 | jkh7iqck | Flat | p2/s42/r1-lag | splay_stand_still | r1-lag/model_3498 | 3498→3997 | 0.45 (4) |
| p2-s42-r1-cont-1500 | e79oihhl | Flat | p2/s42/r1-lag | splay_stand_still | r1-cont-1000/model_3997 | 3997→4496 | 0.50 (4) |
| p2-s42-r2 | 2i9g5098 | Flat | p2/s42/r2-lag | splay_stand_still | r1-lag/model_3498 | 3498→3997 | 0.51 (4) |
| p2-s42-r3 | rrdkrrzw | Flat | p2/s42/r3-lag | splay_stand_still | r2/model_3997 | 3997→4496 | 0.52 (4) |
| **p2-s42-r2-cont-1500 (P)** | **wmhcp24q** | Flat | p2/s42/r2-lag | splay_stand_still | r2/model_3997 | 3997→**4496** | 0.50 (4) |
| p3 Bumpy leg1 (+500) | bhj73f64 | Bumpy | p2/s42/r2-lag | splay_stand_still | P/model_4496 | 4496→4995 | 0.32 (2) |
| p3 Bumpy leg2 (+1000) | 0mnrsqsi | Bumpy | p2/s42/r2-lag | splay_stand_still | leg1/model_4995 | 4995→5494 | 0.32 (2) |
| p3 Bumpy leg3 (+1500) | 4spb61ib | Bumpy | p2/s42/r2-lag | splay_stand_still | leg2/model_5494 | 5494→5993 | 0.36 (2) |
| **p3 Bumpy leg4 (+2000)** | **23pt1499** | Bumpy | p2/s42/r2-lag | splay_stand_still | leg3/model_5993 | 5993→**6492** | 0.35 (2) |
| p3 flat ctrl leg1 (+500) | jtz42coz | Flat | p2/s42/r2-lag | splay_stand_still | P/model_4496 | 4496→4995 | 0.32 (2) |
| p3 flat ctrl leg2 (+1000) | aagiyf2k | Flat | p2/s42/r2-lag | splay_stand_still | leg1/model_4995 | 4995→5494 | 0.31 (2) |
| p3 flat ctrl leg3 (+1500) | clhpdudm | Flat | p2/s42/r2-lag | splay_stand_still | leg2/model_5494 | 5494→5993 | 0.35 (2) |
| **p3 flat ctrl leg4 (+2000)** | **13m4vcia** | Flat | p2/s42/r2-lag | splay_stand_still | leg3/model_5993 | 5993→**6492** | 0.34 (2) |

**Seed-43 replicate line** (same structure, all Flat, splay_stand_still): p1-gallop_c-s43 `y38xkkvh` (scratch) → ctrl-500/1000/1500 `e9wyyh4s` / `2q2wjdt4` / `4r1w7euc` · r1-lag `bcsvvl03` → r1-cont-1000/1500 `l9fv0ek9` / `0jknmncx` · r1-naive `vhbultky` · r2 `v3srag94` → r2-cont-1500 `rwfqzxv2` · r3 `adn6hd33`. Refs are `p2/s43/r{1,2,3}-lag`. Part 3 was not run on s43.

**Totals.**
- The shipped path is 3000 + 1500 + 2000 = **6500 updates**. Its critical chain took about 2.46 + 1.51 + 1.35 ≈ 5.3 h of wall-clock while sharing the GPU (≈ 1.6 h if run solo).
- The whole program trained ≈ 35,900 iterations: Part 1 21,000, Part 2 10,500, Part 3 4,000 plus smokes. That is **≈ 9.3 GPU-h**.

### 1.3 What a warm start carries over and what it resets

Checked in `rsl_rl` 5.4.0 (`OnPolicyRunner.load` → `PPO.load`, default `load_cfg` = everything), in the checkpoints themselves, and in WandB history.

| State | Warm start | Evidence | Consequence |
|---|---|---|---|
| Actor weights, incl. **action-noise std** | **Carried** | `actor_state_dict` has `distribution.std_param`. P's std is 0.21–0.75 per joint; after Bumpy leg 1 it is 0.24–0.89. | Exploration continues where it was; Bumpy raised it (0.37 → 0.43 mean). |
| Critic weights | **Carried** | `critic_state_dict` | The value function starts with the *old* reward and terrain (see Q2.3). |
| **Observation normalizer** (running mean/var, actor and critic) | **Carried and keeps updating**; it is a cumulative average | P's `obs_normalizer.count` = 442,368,000 = 4096 × 24 × 4500, i.e. every sample since scratch. leg1 = 491,520,000. | New data enters slowly. After +N iterations its weight is N/(4500+N): about 10 % after +500 and 31 % after +2000. Large observation shifts (new terrain, shoves) are normalized with mostly old statistics for a while. |
| Adam optimizer moments | **Carried** | `optimizer_state_dict` (21 tensors) | |
| **Learning rate** | **Reset** | `PPO.learning_rate` is rebuilt from config (1e-3). The adaptive-KL rule overwrites the optimizer's saved rate (7.6e-5 at P) on every mini-batch. In every warm-started run, WandB `Loss/learning_rate` reads 1e-5 (the floor) at iteration 0 and ~1e-4 by iteration 10–20 (e.g. bhj73f64, wmhcp24q, fqt1qabi, 2i9g5098). | The first mini-batch steps run at the config rate. The KL then spikes, the rule drops the rate to its floor, and it recovers in ~10 iterations. This happens at **every leg boundary**: the existing 4 × 500 Bumpy +2000 had 4 restarts, while a single +2000 leg has 1. They are not structurally identical. |
| **Iteration counter** | **Carried** | `iter` key. The loop runs from the parent's index, so +500 ends at index +499. | File names continue (4496 → 4995). No curriculum reads it (`curriculum: {}` after the warm-start pop). |
| Env step counter, episode buffers, rollout storage | **Reset** | A new env is built; `init_at_random_ep_len=True` | The first-iteration episode stats are artifacts (mean episode length 19.9 at Bumpy leg1, iteration 0). |
| Pretrain phase (`hold_base_airborne`, `pretrain_rewards`) | **Removed** | `pupper_locomotion.py:140–143` | A warm start goes straight to full rewards. |
| Reward weights, task/terrain, DR ranges, events, seed, reference dir | **Taken from the new launch** (CLI and config), not from the checkpoint | `launch.json` per run | These are free to change at a warm start. The reference must be the one the policy trained on unless the change is deliberate. |

---

## Q2. Rolling back Part 3 Bumpy to a safe point

**Conceptually this is a branch, not a rewind.** Checkpoints are immutable files. "Rolling back" means picking an earlier checkpoint as the warm-start parent of new sibling runs. The existing Part 3 runs stay as evidence and as a ready-made control (see 2.2).

### 2.1 Which checkpoints are safe

Eval seeds are 1234 / 4321 (one value = seed 1234 only). Lower is better.

| Checkpoint | flat_dr falls | bumpy_dr falls | nompd falls | push_stand@0.65 | v50 (m/s) | HOLD a / b / c | Safe? |
|---|---|---|---|---|---|---|---|
| Part 1 parent `p1-gallop_c-s42/model_2999` | 40 | 81 | 99 / 79 | 0.453 | – | gap +48.5 ✗ / – / ✗ | No |
| **P** `p2-s42-r2-cont-1500/model_4496` | 14 / 16 | 35 | 39 / 37 | **0.096 / 0.122** | **0.760** | ✗ (+23) / anchor / anchor | **Push-safe; fails the nominal-gain gap** (which WIDEN training then fixed) |
| Bumpy +500 `model_4995` | 14 / 19 | 31 / 36 | 25 / 31 | 0.798 | – | ✓ / ✓ / ✗ | No (push) |
| Bumpy +1000 `model_5494` | 34 / 39 | 46 / 59 | 42 / 41 | 0.754 | – | ✓ / ✓ / ✗ | No (push, falls dip) |
| Bumpy +1500 `model_5993` | 11 | 23 | 14 | 0.533 | – | ✓ / ✓ / ✗ | No (push) |
| Bumpy +2000 `model_6492` | 10 / 11 | 18 / 23 | 7 / 12 | 0.489 / 0.497 | 0.652 | ✓ / ✓ / ✗ | No (push) |
| Flat ctrl +500 / +1000 | 16 / 10; 18 / 18 | 38 / 41; 47 / 42 | 28 / 23; 34 / 25 | not measured | – | ✓ / ✓ / ? | Unknown |
| Flat ctrl +1500 | 23 | 52 | 51 | 0.024 | – | ✓ / ✗ (51 > 48, one seed) / ✓ | No (nompd) |
| **Flat ctrl +2000** `flatctrl-kpwide-leg4/model_6492` | 12 / 13 | 29 / 34 | 20 / 18 | **0.091 / 0.089** | 0.736 | ✓ / ✓ / ✓ | **Yes, on all four** |

Sources: `part3/diag/README.md`, `part3/eval/p3-final-kpwide_{gates,summary}.md`, `decisions/2026-11-18-w4-p3-candidate.md` (+ addendum), `part3/push_sweep/summary.md`.

- **The Bumpy line has no safe point after P.** Push fails from +500 on, at every checkpoint and neighbour measured (0.49–0.86). The cause is a one-sided standing lean that Bumpy training tripled (`part3/reward_shaping.md`).
- **Recommended rollback point: P.**
  - It is the last checkpoint before the Bumpy branch, and it is push-safe.
  - Its nominal-gain gap is exactly what WIDEN training removed on both arms (flat ctrl +2000: gap +6.5).
  - Every existing Part 3 number is "+N from P", so new arms stay iteration-matched.
  - The reward-shaping sweep already proposes P.
- **Second option: flat ctrl +2000** (Bumpy fine-tuning of the hardware-clear policy). This is the best-rounded checkpoint, but branching from it makes new arms +4000 from P, with no matched data. Use it for a later stage, not for the shaping screen.

### 2.2 Applying a reward change by warm start

- **Yes, a reward change applies by warm start.** Rewards, events, terrain and command mix come from the launch, not the checkpoint (table 1.3). A new reward term does not change the network or the observation layout, so the checkpoint still loads strictly.
- **Some changes are not reward weights.** The standing command share (`rel_standing_envs`) and training shoves (`push_robot`, removed at `pupper_env_cfg.py:302`) need small config flags (`reward_shaping.md`, "Code needed").
- **Free control.** Branching from P lets the existing Bumpy legs serve as the old-reward control at +500/+1000/+1500/+2000: same seed, task, WIDEN ranges and reference. The one mismatch is the leg structure (LR restarts, 1.3). Either run new arms as 500-iteration legs too, or rerun an A0 control as a single leg (`reward_shaping.md` does the latter).

### 2.3 Does changing rewards invalidate the critic?

- **What the critic learns.** The critic estimates the discounted return under the *old* weights and state distribution. With γ = 0.97 its horizon is ~33 control steps (≈ 0.67 s at 0.02 s per step), so re-fitting is a short-horizon problem.
- **Measured precedents.** Value loss at iteration 0 → iteration 5 (WandB `Loss/value`):

  | Switch | Run | Iteration 0 | Iteration 5 | Steady band |
  |---|---|---|---|---|
  | Two stand-still terms added (`splay` → `splay_stand_still`) | ctrl-500 fqt1qabi | 0.30 | 0.14 | 0.24–0.28 |
  | Same weights, new reference only | r2 2i9g5098 | 0.32 | 0.13 | ~0.25 |
  | Flat → Bumpy terrain | Bumpy leg1 bhj73f64 | **1.08** | 0.28 | 0.35–0.44 |

  The critic re-fits within about 5–10 iterations. It is **not invalidated** by changes of this size.
- **When it would matter.** Large changes cause it:
  - `roll_asymmetry` ×10;
  - standing share 0.02 → 0.2 (a new state distribution);
  - shoves that create falls the critic has never valued.

  For the first tens of updates, the advantages are then biased, and the actor may move in a wrong direction. The +500 transients seen in Part 3 are of that kind.
- **Mitigations, cheapest first:**
  1. **Watch.** Pre-register value loss and KL over the first 50 iterations against A0. If iteration-0 value loss is > 3× A0's, add mitigation 3.
  2. **Ramp the change.** Ramp shove speed and new weights linearly over the first ~200–300 iterations. Shoves are already planned as ±0.5 → ±0.8 m/s.
  3. **Critic warm-up.** Freeze the actor (actor LR 0, or skip the surrogate loss) for 50–100 iterations so the critic fits the new returns first. That costs ~1–2 min. rsl_rl has no flag for this: actor and critic share one optimizer, so it is a small code change.
  4. Do **not** re-initialize the critic (`load_cfg={"actor": True}`). A critic from scratch needs hundreds of iterations.

  Per-batch advantage normalization already absorbs overall reward-scale changes.

### 2.4 How much fits in 45 minutes

**Budget per window:** about 1 min startup (env build ~30–40 s), ~37 min training, ~5–7 min evals (flat_dr + bumpy_dr suites ~3 min, probes < 4 min each, push ~2 min).

| Arms run together | Per-arm rate (Bumpy) | Iterations per arm in ~37 min | Matched to existing data at |
|---|---|---|---|
| **1** | ~1.0 it/s (solo 0.86 s/it flat × 1.15) | **~2,200 → +2000** | +2000 (candidate, flat ctrl) |
| 2 | ~0.45 it/s (measured 2 × 500 in 19–21 min) | ~1,000 → +1000 | +1000 (the documented dip; a poor readout) |
| 4 (cap) | ~0.22 it/s | ~500 → +500 | +500 (exact structure match to Bumpy leg1) |

- **The concurrency cap of 4 buys nothing.** The trainer is bound by one CPU thread per process, but the GPU time-slices the processes. Two concurrent processes give 113k vs 125k env-steps/s solo, about −10 % in total (`infra/gpu_bottleneck.md`). Every row above delivers about the same ~2,000–2,300 iterations per window.
- **Choose by readout point, not by "parallelism".**
  - The planned **6-arm sweep at +2000** (`reward_shaping.md`) runs one arm per window. That is ≈ 3.7 GPU-h, about 4–4.5 h of wall-clock.
  - **Faster two-stage alternative.** **Stage 1** screens 4 arms at +500 per window (2 windows ≈ 1.5 h). The push regression was already fully visible at +500 (0.80), so the readouts are the push, lean and action-rate metrics. Pre-register a kill rule: push_stand@0.65 > 0.5 at +500 → drop. **Stage 2** takes the top 2 to +2000 (2 windows ≈ 1.5 h).

    Total ≈ 3 h, ≈ 2.6 GPU-h. Its caveat is that falls at +500 are noisy (checkpoint-to-checkpoint ±7–10 falls), so use +500 for push and posture only.

### 2.5 Branch vs. retrain from scratch

| | Branch from P (warm start) | Retrain from scratch with new rewards |
|---|---|---|
| Iterations per arm to reach "Part 3 +2000" | 2,000 | 3,000 (Part 1) + 1,500 (Part 2 chain) + 2,000 = 6,500, plus 2 capture/fold rounds |
| GPU-h per arm | ≈ 0.6 | ≈ 1.75 + captures |
| Wall per arm, solo | ≈ 40 min (fits 45) | ≈ 2 h (Part 1 alone is ~45–50 min solo, so it does not fit) |
| 6-arm sweep | ≈ 3.7 GPU-h, 4–4.5 h | ≈ 10.5 GPU-h, 11–12 h |
| Attribution | One change on top of a fixed, evaluated lineage | The change also acts during Part 1/2, with no matched data. Parts 1–2 conclusions must be re-made |
| When to prefer it | Reward weights, events, terrain, DR, command mix | The change alters what Parts 1–2 learn: the reference or clock, the observation layout, the network, `num_envs` |

---

## Q3. The "fusion" experiment: gallop C + faster clock + Bumpy + DR + training shoves

**Current clock:** gallop C at **2.25× (3.0 strides/s, period 0.333 s)**. The 2.5× proposal gives 3.33 strides/s (0.300 s), +11 %.

**What a new clock invalidates.**
- The clock lives in the reference manifest, and the policy observes the reference.
- The Part 1 parent and every Part 2 capture are clock-specific: a capture records how *that* policy moved at *that* clock.
- The export also carries the clock to the robot (`gallop_freq_mult` in `policy.json`).

**Pre-checks (CPU-only, computed here by scaling ×2.5/2.25):**

| Table | kp·Δq (Nm), now → at 2.5× | Peak joint speed (rad/s), now → at 2.5× |
|---|---|---|
| r2-lag (the Part 2 reference) | 2.24 / 2.45 → **≈ 2.5 / 2.7** (under the 3 Nm fold gate) | 21–23 → **≈ 24–26** |
| gallop C design table | 3.82 → **≈ 4.2** (was already above 3) | 44.8 → **≈ 50** |

gallop C already has the highest 3 Nm saturation of the Part 1 arms (≈ 16–18 % at ±1.5 m/s), and a faster clock will push it up. Commands stay capped at ±1.5 m/s, so at equal speed a faster clock means shorter strides, not a faster robot.

### 3.1 Full rerun: every Part at 2.5× (the strict answer: yes, all Parts)

Two seeds, decision sets only (the report-only arms from last time are reused, not rerun).

| Step | Must rerun? | Iterations | Training GPU-h | Wall-clock with critical-path scheduling |
|---|---|---|---|---|
| 0. Author gallop_c@2.5 (`bootstrap.py design --frequency-mult 2.5`), table checks, commit + push | yes (new reference) | 0 | 0 | ~0.5 h agent time, overlapped with step 0 of 3.4 |
| 1. Part 1: gallop_c@2.5 × {s42, s43} from scratch, 3000 each. The 2.25× s42/s43 parents already exist as comparators | **yes** | 6,000 | 1.56 | s42 solo first: **0.75 h** to the s42 parent. s43 fills gaps |
| 2. Part 1 gate (eligibility: runs ±1.5 both ways; sat@1.5, nompd, push not worse than the 2.25× parent by > band) | yes | – | ~0.1 eval | 0.25 h, rule pre-registered |
| 3. Part 2 decision set per seed: capture → r1-lag (+500) → capture → r2 (+1000) → r2-cont (+1500), plus ctrl-500/1000/1500 | **yes** (captures are clock-specific). Can skip r1-naive, backpin, r1-cont and r3 (settled last time) | 6,000 (12 legs) | 1.56 | Chain = 3 legs + 2 captures: **≈ 1.0 h** for s42 at priority. Total work sets the pace |
| 4. Part 2 final gate (two-seed, 4 lenses) | yes | – | ~0.3 eval | 0.5 h |
| 5. Part 3 fusion: nompd step 0 + smoke, then **fusion arm** (Bumpy + DR/WIDEN + shoves + winning shaping) and a **matched flat control** at the same clock, +2000 each in gated 500-iteration legs | **yes** | 4,000 (+100 smoke) | 1.15 | ≈ 2.0 h (measured last time: train + gates 2.0 h for this exact structure) |
| 6. Select, confirm on seed 4321, HOLD a/b/c, export | yes | – | ~0.2 eval | 0.3 h |
| **Total** | | **≈ 16,000** | **≈ 4.3** (+ ~1 GPU-h of evals) | **≈ 5.5–6 h** scheduled; **≈ 8 h** with last run's ordering |

- **Why scheduling matters here.** The total GPU work (≈ 5.3 h with evals) exceeds the dependency chain (≈ 2.7 h: s42 Part 1 solo, then the s42 Part 2 chain, then Part 3, then gates). So the GPU's total work sets the end time, as long as the queue never idles and the s42 chain always has priority.
- **Single-seed variant (s42 only).** About 10,000 iterations, 2.7 GPU-h, ~3.5–4 h. It **breaks the two-seed rule** that both Part 2 decisions relied on.

### 3.2 Cheaper variant: re-clock and warm-start (skip Parts 1–2)

| Step | Iterations | Training GPU-h | Wall |
|---|---|---|---|
| 0. Copy `p2/s42/r2-lag` with `frequency_mult` 2.5 (same table, new manifest and sha). Run the fold-check table checks, commit + push | 0 | 0 | 15 min |
| 1. **Fusion arm:** warm-start P on the re-clocked reference, Bumpy + WIDEN + shoves + winning shaping, +2000 as a single leg | 2,000 | 0.6 | one 45-min window |
| 2. **Matched control:** the same recipe at 2.25× = the shaping sweep's winning arm (already run) | 0 | 0 | – |
| 3. *Optional* clock-only diagnostic: flat, 2.5×, +2000 from P vs the existing flat ctrl +2000 | 2,000 | 0.5 | one window; can run during the sweep |
| 4. *Optional* one bootstrap round at 2.5×: capture the fusion arm, fold, +500 | 500 | 0.15 | 20 min |
| 5. Evals, two seeds, HOLD a/b/c, push sweep, export | – | ~0.2 | 20–30 min |
| **Total** | **2,000–4,500** | **≈ 0.6–1.3** | **≈ 1–1.5 h** after the shaping sweep |

- **Cost of the shortcut.**
  - It is a **deviation**: the 2.5× policy never goes through Part 1/2 at its own clock.
  - Part 2's references come from a 2.25× policy, so they are no longer self-consistent at 2.5×. Optional step 4 partly restores that.
  - Part 1/2 claims stay about 2.25×.
  - The precedent that the shortcut can work: Part 2 legs already swapped references under a warm start without collapse, once the out-of-range channels were pinned.
- **When the full rerun is required.** Only if the report must show Parts 1→2 at 2.5×.

### 3.3 "Ducks in order" before deciding

1. **Push fix and reward shaping first.** Run the `reward_shaping.md` sweep (A0–A5, or the two-stage version in 2.4) and pick a recipe. Fusion stacks four changes (clock, terrain, DR, shoves). Without a settled recipe, any result is unattributable.
2. **Human calls pending.**
   - Ratify or reject HOLD(c).
   - Choose the hardware option (flat ctrl +2000 or hold).
   - Rule on whether "fusion" must go through Parts 1–2 at 2.5× (full rerun) or may be a labelled Part 3 deviation (cheap variant).
3. **State the hypothesis and its metric.** Why 2.5×? For example: "shorter, more frequent steps improve shove recovery and Bumpy falls at equal speed". The pass metric would be v50 and bumpy_dr. Top speed cannot rise while commands are capped at ±1.5 m/s.
4. **Matched controls, one per factor, fixed before launch.**
   - Fusion vs the shaping winner at 2.25×: the clock.
   - Optional flat 2.5× vs flat ctrl +2000: the clock without terrain.
   - Same seed, `num_envs` 4096, the same leg structure (single leg vs 4 × 500: LR restarts), and the same reference version.
5. **Eval plan, pre-registered.**
   - flat_dr, bumpy_dr and nompd on two seeds.
   - push_stand@0.65 on two seeds (HOLD c), and push split by side.
   - Standing posture and action-rate probes (`reward_shaping_probe/`).
   - **Push at nominal gains**: never measured for any policy.
   - sat@1.5 and foot force, which the clock raises.
   - The seam test, and a v50 sweep for finalists.
   - Thresholds and guardrails as in `reward_shaping.md` ("How to score"). Kill rules written before launch.
6. **Code flags merged and smoke-tested.**
   - `rel_standing_envs`.
   - Linear-only `push_robot` with a ramp.
   - Single-leg length.
   - Re-clock helper.
   - Optional critic warm-up (2.3).
   - Check that the export carries `gallop_freq_mult` 2.5 and that the C++ gait test passes at 2.5×.
7. **Scheduling.**
   - One +2000 arm per 45-min window.
   - Evals in their own windows: a co-scheduled eval slows a trainer by ~27 %.
   - Priority queue on the decision set, and event-driven hand-off at each gate (`infra/wallclock_retro.md`).
8. **Noise rules applied.**
   - Fall gaps < 10–15 are ties, plus ±7–10 from checkpoint-to-checkpoint variance.
   - Push run-to-run spread is ±0.02.
   - Second training seed only for an arm that passes; s43 has its own P, `rwfqzxv2`.

## Sources

- `report/REPRO_PLAN.md`, `report/decisions/*.md`, `report/infra/{wallclock_retro,gpu_bottleneck,a100_throughput}.md`.
- `report/part3_notebook.md` §0, `report/part3/diag/README.md`, `report/part3/eval/p3-final-kpwide_*.md`, `report/part3/push_sweep/summary.md`, `report/part3/reward_shaping.md`.
- `scripts/opt_lab_2/{launch,part1_queue,part2_queue,part3_finetune,part3_run}.py` and `src/mjlab/scripts/pupper_locomotion.py` (train).
- `rsl_rl` 5.4.0 (`runners/on_policy_runner.py`, `algorithms/ppo.py`).
- `~/runs/opt-lab-2/*.launch.json`, run logs and `params/*.yaml`, checkpoint contents (`model_4496.pt`, `model_4995.pt`).
- WandB project `mjlab` (`Loss/learning_rate`, `Loss/value`).
- The Claude Doc "Opt Lab 2 Reproduction — Manager Report" and its Summary-tab comment threads.
