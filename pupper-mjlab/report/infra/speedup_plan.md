# Faster PPO training: is there a quick fix, and what to change

Written 2026-11-19 on branch `markus/opt-lab-2`. It follows [`gpu_bottleneck.md`](gpu_bottleneck.md): at 4096 envs, one CPU thread bounds training, not the GPU. The hardware and stack are the same as there: 1× A100-PCIe-40GB, torch 2.9.0+cu128, warp 1.14.0, mujoco-warp 3.10.0.2, mjlab 1.5.2 (fork), rsl-rl-lib 5.4.0.

**Status: plan plus short prototype measurements.** No mjlab source was changed and no training was run. Each benchmark below ran for under 5 min through `scripts/opt_lab_2/gpu_slot.py`, with only the two idle viewer processes also on the GPU. One earlier run was contaminated by another agent's job, so it was discarded and rerun.

## Answer

- **No config flag, compile flag, upstream release or open PR gives more than about 4 %.** The fixes that cost nothing and do not change results add up to **−4 % wall time per iteration** (measured: 0.794 → 0.761 s/iter). `torch.compile` on the policy made it slower.
- **The real gain needs a code change: run the reward, observation, command and termination code as CUDA graphs, and do the same for the PPO update.** The estimate is **about 1.5× (range 1.4–1.75×)** at equal iterations, with unchanged PPO math. The effort is about 3–4 days. Only the first step is measured. The rest is estimated from per-segment timings.
- At 4096 envs, physics is not the problem. It already runs as a CUDA graph and costs 7.3 ms of each 24 ms env step, all of it GPU time.

## Terms

| Term | Meaning here |
|---|---|
| Host / device | The CPU running Python / the GPU |
| Kernel, launch | One GPU function call. "Launching" it costs the host roughly 5–20 µs of Python and driver time, however small the work |
| Sync (host–device synchronization) | The host stops and waits until the GPU has finished all queued work, usually to read a value back. While it waits, it cannot queue new work |
| Eager mode | Normal PyTorch: every op is launched from Python, one by one |
| CUDA graph, capture, replay | Record a fixed sequence of kernels once ("capture"), then launch the whole sequence with one call ("replay"). The kernels and their inputs' memory addresses are frozen, so captured code must have fixed tensor shapes, no syncs and no Python branches on GPU values |
| `torch.compile` | PyTorch compiler that fuses small ops into fewer kernels |
| Iteration | One PPO cycle: 24 env steps × 4096 envs of data collection ("collect"), then 5 epochs × 4 mini-batches of gradient steps ("learn") |
| Env step | One policy step = 5 physics substeps plus termination, reward, reset, command, event and observation code ("managers") |
| ABAB | Alternate baseline (A) and variant (B) blocks in one process, so that GPU clock or thermal drift hits both equally |
| Floating-point (fp) noise | Differences of about 1 part in 10⁷ from doing the same math in a different order. Bitwise identical means exactly zero difference |

## 1. Candidate quick fixes: verdict per candidate

Gains are wall-time change per iteration at 4096 envs, against the baseline of 0.794 s/iter (median of 20 iterations, ABAB). "Measured" means this prototype measured it. "Est." means it is derived from the segment timings in §3.

| Candidate | Evidence | Gain | Effort | Risk to results | Verdict |
|---|---|---|---|---|---|
| CUDA graphs for physics (mjlab `SimulationCfg`) | Already on: `sim.py` captures `step`/`forward`/`reset`/`sense`. There is no off-switch to flip | 0 (already in use) | — | — | Nothing to gain |
| `graph_conditional` option ([mjlab#1201](https://github.com/mujocolab/mjlab/pull/1201), merged after our pin) | Fixes a slow-H100 edge case. Physics here is GPU-bound and fast (1.4 ms/substep) | ≈ 0 | — | Physics unchanged | Not relevant |
| **Index writes without syncs** (`t[ids] = 0.0` → `t.index_fill_(0, ids, 0.0)`) | 42 call sites cause **1,598 of 2,410 syncs** per iteration. Open upstream issue [mjlab#1296](https://github.com/mujocolab/mjlab/issues/1296) proposes exactly this; there is no PR | Measured −3 to −4 % (with the next two rows) | 2 h | None: bitwise-identical values | **Do (step 1)** |
| Return episode logs as tensors, not `.item()` | Termination and command reset logs: 96 syncs/iter. The rsl_rl logger: 120 syncs/iter | Included above | 1–2 h | None: logged values identical | **Do (step 2)** |
| NaN check once per rollout instead of every step | rsl_rl 5.4 runner already reads `check_for_nan`. mjlab's config does not expose it ([mjlab#1270](https://github.com/mujocolab/mjlab/pull/1270), closed unmerged). 96 syncs/iter | Included above | 1 h | None: same check, ≤ 24 steps later | **Do (step 3)** |
| Fused Adam optimizer | One fused kernel instead of a loop over parameters | Measured −0.7 % (learn −8 ms/iter) | 0.5 h | fp noise only | **Do (step 4)** |
| `torch.compile` actor/critic (rsl_rl `torch_compile_mode="default"`) | Fails out of the box: mjlab's `configure_torch_backends` breaks inductor on torch 2.9 (we hit the exact error that open PR [mjlab#1285](https://github.com/mujocolab/mjlab/pull/1285) fixes). With that workaround, outputs are bitwise identical, but | Measured **+1.9 % slower** (0.761 → 0.775) | — | None | **Don't** |
| Upgrade mjlab 1.5.2 → 1.6.1 / mujoco-warp 3.10 → 4.0.1 | Release notes ([mjlab](https://github.com/mujocolab/mjlab/releases), [mujoco_warp](https://github.com/google-deepmind/mujoco_warp/releases)) make no throughput claim. mujoco_warp solver PRs speed up physics only, which is 21 % of wall time at 4096 | ≤ 3 % (est.) | 1 day+ | **Changes physics**: new lineage | **Don't** (not for speed) |
| Rejected upstream perf PRs ([mjlab#1061](https://github.com/mujocolab/mjlab/pull/1061) cache hot paths, [#1342](https://github.com/mujocolab/mjlab/pull/1342) device tensor ids) | The maintainer declined #1061 for complexity. Neither targets our dominant syncs | small | — | — | Upstream won't carry them; keep our patches small |
| More processes, MPS, larger `num_envs`, fewer mini-batches | See `gpu_bottleneck.md` | — | — | Process count: none, but no gain. Others change PPO numerics or are ruled out by policy | Out of scope |

## 2. Where the 2,410 syncs per iteration come from

Measured with `torch.cuda.set_sync_debug_mode("warn")` over 2 iterations at 4096 envs. This mode flags every op that blocks the host, including hidden ones. `gpu_bottleneck.md`'s ≈ 2,600 also counts driver-level syncs.

| Cause | Syncs/iter | Sites | Example | Fix |
|---|---|---|---|---|
| Scalar write through an index tensor | **1,598** | 42 | `reward_manager.py:111 self._episode_sums[key][env_ids] = 0.0` (528/iter: one per reward term per step) | `index_fill_` / `masked_fill_` (step 1) |
| Python `if` on a GPU value | 207 | 8 | `if singular.any():` (raycast sensor), `if kl_mean > ...` (PPO adaptive LR) | Branch-free `torch.where` (step 5) or keep |
| `.item()` / `.cpu()` readback | 204 | 7 | `termination_manager.py:97`, `command_manager.py:103`, rsl_rl logger | Tensors, read once per iteration (step 2) |
| `nonzero()` | 120 | 5 | reset ids in `env.step` (needed), standing / heading-capture ids in velocity commands | Masks (step 5); keep the reset one |
| `torch.normal` std check | 88 | 2 | policy action sampling (`distribution.py:195`) | Leave: a fix changes the random draws |
| Boolean-mask indexing, host list → device | 120 | 5 | `env_ids[mask]`, `torch.tensor(range_list)` in `events.py:40` | Masks; cache the range tensor |

By phase: env.step 2,006, PPO update 139, logger 120, NaN check 96, policy act 48. After steps 1–3: **885 syncs/iter** (measured).

## 3. Where the time goes in one env step

Each segment was timed with a GPU sync before and after it, over 48 steps at 4096 envs. "Host" is the time to issue the segment. Where host ≈ synced, the segment is host-bound: the GPU waits on Python.

| Segment | Host ms | Synced ms | Bound by | Graph-capturable as is? |
|---|---|---|---|---|
| Physics: action → 5 × (actuators, `sim.step`, sensors) | 4.1 | **7.3** | GPU | Already graphed (physics). No gain from capturing more (tested: 7.3 vs 6.3 ms) |
| `_reset_idx` (resets every step; variable env count) | **5.5** | 5.5 | host | No: variable shapes and random draws per reset env |
| Observation manager | **4.4** | 4.4 | host | After step 5 |
| Command manager | 1.7 | 1.7 | host | Update part yes. Resample stays eager |
| Reward manager | 1.7 | 1.7 | host | Yes, after step 1 |
| `sim.sense` (raycast prep + warp graph) | 1.4 | 1.6 | host | Torch part after removing `.any()` |
| `sim.forward` | 0.1 | 1.2 | GPU | Already graphed |
| Events, termination, metrics, reset-id `nonzero` | 1.0 | 1.0 | host | Yes / keep the one `nonzero` |
| **Total** | | **≈ 24.7** | | ≈ 9.9 ms/step host-bound and capturable → **≈ 0.24 s/iter** |

The learn phase is 0.19 s/iter and host-bound as well: 20 optimizer steps at about 9 ms each, of which autograd bookkeeping alone is 0.10 s/iter.

## 4. Plan, ranked (quick wins first)

| # | Step | Files / functions | Gain (vs 0.794 s/iter) | Effort | Changes results? |
|---|---|---|---|---|---|
| 1 | Sync-free index writes | See list below (42 sites) | measured −3 % with 2+3 | 2 h | No (bitwise) |
| 2 | Episode logs as tensors | `managers/termination_manager.py: reset`, `managers/command_manager.py: CommandTerm.reset`, logger subclass in `rl/runner.py` | in #1 | 1–2 h | No (logging only) |
| 3 | NaN check once per rollout | `rl/config.py: RslRlBaseRunnerCfg` add `check_for_nan: bool = False`; `rl/vecenv_wrapper.py: step` keeps a device-side `nan_flag`, checked every `num_steps_per_env` steps | in #1 | 1 h | No |
| 4 | Fused Adam | `rl/runner.py: MjlabOnPolicyRunner.__init__`: set `fused=True, foreach=False` on `alg.optimizer.param_groups` before the first step | measured −0.7 % | 0.5 h | fp noise |
| 5 | **CUDA-graph the fixed-shape manager code** | `envs/manager_based_rl_env.py: step` (new `_graph_post_physics`, `_graph_pre_obs`); sync-free rewrites in `tasks/velocity/mdp/velocity_command.py: _update_command`, `tasks/pupper/mdp/velocity_command.py: _update_command`, `sensor/raycast_sensor.py: _extract_yaw_rotation`; stable buffers in `envs/mdp/actions/actions.py: process_actions`, `tasks/pupper/mdp/actions.py: process_actions`; opt-in flag on `ManagerBasedRlEnvCfg` | est. **−0.20 s (−25 %)** | 12–16 h | No (same kernels; verify bitwise) |
| 6 | **CUDA-graph the PPO mini-batch step** | New `PPO` subclass in `rl/` (forward, losses, backward, grad clip, `Adam(capturable=True)`; adaptive LR computed on device; loss sums read once per iteration) | est. **−0.12 s (−15 %)** | 6–10 h | fp noise (same math) |
| 7 | *Needs your decision:* fixed-shape masked reset, then graph `_reset_idx` | `_reset_idx`, all reset events, `command_manager._resample` | est. −0.05…−0.10 s | 8–16 h | **Yes, random draws change.** Same distributions, different samples for a given seed, so this starts a new lineage |

**Expected total, steps 1–6:** 0.794 → about **0.45–0.55 s/iter (1.45–1.75×)**, that is 125 k → about 180–215 k env-steps/s. It cannot go below the GPU busy time, ≈ 0.38 s/iter (`gpu_bottleneck.md`). Physics then becomes the largest share. Steps 1–4 are measured (0.761 s/iter). Steps 5–6 are estimates and carry the main uncertainty.

### Step 1 sites (`t[ids] = scalar` → `t.index_fill_(0, ids, scalar)`; column forms → `t[:, k].index_fill_(0, ids, v)` / `t.index_fill_(1, ids, v)`)

Keep the existing `None`/`slice` branches; only the tensor-index path changes. A small helper `mjlab.utils.torch.fill_rows_(t, ids, value)` keeps the call sites readable.

- `managers/reward_manager.py: reset` (528/iter), `metrics_manager.py: reset`, `event_manager.py: apply` (reset mode, 2 lines), `action_manager.py: reset` (3), `command_manager.py: reset`
- `envs/manager_based_rl_env.py: _reset_idx` (2), `envs/mdp/actions/actions.py: reset`, `entity/data.py: clear_state` (7), `sensor/contact_sensor.py: reset` (4)
- `utils/buffers/circular_buffer.py: reset` (2), `utils/buffers/delay_buffer.py: reset` (2), `sim/sim.py: reset` (`_reset_mask[env_ids] = True`)
- `tasks/velocity/mdp/velocity_command.py: reset, _resample_command, _update_command`; `tasks/pupper/mdp/velocity_command.py: _update_command, _resample_command`; `tasks/pupper/mdp/latency.py: reset`; `tasks/pupper/mdp/actions.py: reset`
- `envs/mdp/events.py: _sample_se3_range`: build the `ranges` tensor once and cache it

### Step 5 details (the main work)

- **Two graphs per env step:**
  - **G1** runs after physics: termination, reward and metrics.
  - **G2** runs after `sim.forward()`: command update, step/interval events (fixed-shape part), raycast prep, observations.
  - Reset ids `nonzero`, `_reset_idx`, command resample and the warp graph launches stay eager between them. That leaves 1 sync per step plus the resample.
- **Prototype findings (tested 2026-11-19):**
  - Torch pieces capture fine: `scene.update` went 0.30 → 0.07 ms, `write_data_to_sim` 0.36 → 0.11 ms, `process_action` 0.088 → 0.025 ms.
  - A warp graph cannot be replayed inside a torch capture (Warp error 900). Launch warp graphs between torch graphs, or capture the raw `mujoco_warp` call with `wp.capture_begin(stream=…, external=True)`. That worked.
  - Capture on warp's own stream (`torch.cuda.ExternalStream(wp.get_stream().cuda_stream)`). `sim_data.TorchArray.__setitem__` writes on that stream, and any other capture stream invalidates the capture.
- **Audit before capturing.** Captured code freezes Python state, so check for:
  - Python counters or `if`s that change between steps (e.g. `metrics_manager._substep_count`, `common_step_counter` use inside terms, curriculum).
  - Attributes reassigned to new tensors (`self._processed_actions = …`, `self._delayed_actions = …`). Make these in-place copies.
  - Any remaining sync. Run the step-1 sync-debug count; it must be 0 inside G1/G2.
- Re-capture after anything that reallocates buffers (`expand_model_fields`, curriculum weight changes if they rebuild tensors). Keep an eager fallback behind the opt-in flag. Default off until the checks in §5 pass.

## 5. How to verify equivalence

| Level | Test | Pass criterion | Applies to |
|---|---|---|---|
| L0 unit | For each changed function, compare old vs new on random inputs (pytest, CPU and CUDA) | `torch.equal` (bitwise) | 1, 2, 5 |
| L1 lockstep | Two envs, same seed, same actions, RNG state restored before each step. After every physics step, copy env A's full warp `Data` into env B, so physics noise cannot enter. Compare manager outputs (obs, reward, done, episode logs) | Bitwise equal for 200 steps, with resets forced by randomizing `episode_length_buf` | 1, 2, 3, 5 |
| L2 envelope | Free-running envs A, A2 (baseline twice) and B (new). Per-step max abs difference of obs and reward | B–A within the A2–A spread | all |
| L3 learning | Matched iterations: 2 seeds × {baseline, new} × 300 iterations (≈ 16 GPU-min serial). Compare mean episode reward and length at iterations 100/200/300 | Difference ≤ seed-to-seed spread at every checkpoint | 4, 6 (and a final check of all) |

- **Why L1 and not plain same-seed runs:** MuJoCo-Warp is not bitwise deterministic run to run ([mujoco_warp#562](https://github.com/google-deepmind/mujoco_warp/issues/562)). Measured here: two baseline envs with identical seed, actions and RNG state already differ by 0.02 (max abs, obs) at step 0 and 0.1–0.6 within 60 steps (1024 envs). The step-1–3 patch stayed inside that spread (L2). The L1 lockstep copy removes this noise, so the manager code can be checked bitwise.
- L3 is a training run, so it needs your go-ahead. It follows the matched-iteration rule: compare at equal iterations only.

## 6. How to measure the speedup

- **Tool:** `scripts/opt_lab_2/gpu_bench.py`, which runs the production env and runner with a copy of `OnPolicyRunner.learn`. Use 4096 envs, 5 warm-up + 30 timed iterations, and report medians. Launch through `gpu_slot`. Before and after each run, record `nvidia-smi --query-compute-apps`, and discard any run that overlaps a foreign GPU process.
- **Order:** ABAB in one process when the change can be toggled (steps 1–4 and 5 via its flag). Otherwise, alternate separate processes, 2 each.
- **Report:**
  - s/iter, env-steps/s, collect/learn split
  - syncs/iter (`torch.cuda.set_sync_debug_mode` count; target < 100 in collect after step 5)
  - GPU busy % (`--profile-iters 2`)
  - the §3 segment table again
- **Production check:** the first real run on the new code logs s/iter over its first 50 iterations. Compare that with `rp-p1-fast_trot-s43` (0.86 s/iter solo incl. W&B).
- **Success bar:** ≥ 1.3× at 4096 for steps 1–6. If step 5 alone gives < 10 %, stop and re-plan before step 6.

## Appendix: prototype numbers and scripts

| Variant (ABAB, 2 × 10 iterations each) | s/iter | collect s | learn s | syncs/iter |
|---|---|---|---|---|
| Baseline | 0.794 | 0.600 | 0.194 | 2,587 |
| Steps 1–3 (index writes, tensor logs, no per-step NaN check) | 0.767 | 0.571 | 0.195 | 885 |
| + fused Adam (step 4) | **0.761** | 0.574 | 0.186 | 885 |
| + `torch.compile` actor/critic | 0.775 | 0.579 | 0.197 | — |

- Prototype scripts are in the session scratchpad (not durable): `fastpatch.py` (global `Tensor.__setitem__` → `index_fill_` shim, for measurement only, not for production), `variants2.py` (ABAB + segment timing), `syncmap.py` (sync attribution), `diag_capture.py` (graph-capture feasibility), `equiv.py` (L2 envelope).
- The step 1–4 numbers can be reproduced by re-implementing them from this page: each is a few lines.
