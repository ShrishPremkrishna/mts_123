# Opt Lab 2, Part 0: Spelunking the gait-reference stack

Repo: our fork (F), `pupper-mjlab/`. All `path:line` references are relative to `pupper-mjlab/` and were checked against the current files on 2026-11-18.
Abbreviations: `gr` = `src/mjlab/tasks/pupper_gait/mdp/gait_reference.py`, `gait` = `.../mdp/gait.py`, `mix` = `.../mdp/mixed_gaits.py`, `cfg` = `.../pupper_gait_env_cfg.py`, `exp` = `.../export.py`, `boot` = `.../bootstrap.py`, `loco` = `src/mjlab/scripts/pupper_locomotion.py`.
Each claim carries a tag. **[R]** means verified by reading the code. **[S]** means confirmed by a CPU-only snippet (the stock `.venv` python; `gr` was loaded with `importlib` so the task registry never ran; no GPU and no simulation step; MuJoCo was used only to compile the model for joint limits).

## 1. Main account

**Parametric pipeline [R].** `build_joint_reference_table(n_samples=100, gait)` (`gr:386`) turns four per-gait dictionaries into a table:
- `_GAIT_TOUCHDOWN` (`gr:162`)
- `_GAIT_STRIDE` (`gr:168`)
- `_GAIT_STANCE_Z` (`gr:194`)
- `_GAIT_SWING_LIFT` (`gr:203`)

Two optional dictionaries also feed it: `_GAIT_SWING_KF` (`gr:176`) and the X/Y shifts (`gr:181`, `gr:184`). The steps are:
1. **Keyframes.** `_base_cycle` (`gr:298-332`) builds five stance points `x = front, .5·front, 0, -.5·back, -back` at `z = stance_z` (`gr:314-320`). It then adds swing points:
   - with `n_swing == 1`, a single point `[0, 0, z + lift]` (`gr:322`), giving duty factor 4/6;
   - otherwise `n_swing` points on a sine arc (`gr:324-331`).
2. **Phase.** For sample `i`, `t = i / n`. Each leg reads `(t - touchdown[leg]) % 1` (`gr:468-470`) through `_interpolate_triangle` (`gr:335-342`), which linearly interpolates the looping keyframes.
3. **Body frame.** The leg's `_HIP_OFFSET` (`gr:139-141`) and the optional shifts are added (`gr:471-473`).
4. **IK.** `_ik_single_leg` (`gr:345-363`) uses central-difference gradient descent with `lr = 10`, `max_iter = 100`, `tol = 1e-4` (mean absolute error), `eps = 1e-3` (`gr:352`).
   - It warm-starts from the previous sample's solution (`gr:476`); the first sample starts from `zeros(3)` (`gr:466`).
   - It never checks the residual, so a solve that fails to converge goes through silently.
5. **Output.** The result is `(n, 12)` float64 in `JOINT_NAMES` order (FR, FL, BR, BL × `_1`, `_2`, `_3`; `src/mjlab/asset_zoo/robots/pupper_v3/pupper_constants.py:40-45`), cached per `(n, gait)` (`gr:366`, `gr:477`).
   - **The parametric path is not clamped in NumPy.**

**Shipped parameters [R].**
- trot (`gr:163-205`): touchdown `(0, .5, .5, 0)`, half-stride 0.05, stance_z -0.14, lift 0.09, 1 swing keyframe.
- lift (`gr:256-259`): the same, except stride 0.0.
- `gallop` and `cheetah` exist only in comments.

**Clamp and validation sites (all of them).**

| # | Where | What | Behaviour |
|---|---|---|---|
| 1 | `gr:380-383` `_clamp_to_joint_limits` | `np.clip` to `get_spec().compile()` joint ranges | Applied **only** to jump (`gr:409`) and spider (`gr:413`) |
| 2 | `gait:74-87` `_reference_table` | `torch.clamp` to `robot.data.joint_pos_limits[0]` (hard `jnt_range`) | Applied to **every** table the env, reward or exporter uses. This is the clamp that matters for trot and lift |
| 3 | `gr:370-378` `_validate_capture` | Requires `ndim == 2`, 12 columns, `N >= 8`, all finite, and within limits ± 1e-6 | **Rejects** with `ValueError` and never clips. Used for reach, reach_back and jump |
| 4 | `boot:23-38` `validate_reference` | Same limit test, plus an optional peak-velocity check | Rejects ("repair it rather than silently clipping"). Run by `save_references` (`boot:158`) |
| 5 | `exp:196-199` | Records `clamp_lower` and `clamp_upper` in the deploy JSON | Audit only. The tables are shipped already clamped |

[S] The shipped **trot and lift break the front-knee limits before clamp #2**:
- `leg_front_r_3` reaches 0.802 rad (trot) and 0.806 rad (lift) against an upper limit of 0.71. `leg_front_l_3` mirrors this.
- That is 4 of 100 rows per knee, up to 0.096 rad out (range at `src/mjlab/asset_zoo/robots/pupper_v3/xmls/pupper_v3.xml:55`).

So the hand-written gaits are silently *clipped*, while student captures are *rejected*.

**Capture sockets [R + S].**
- **Directory.** It is resolved at import time: `PUPPER_REFERENCE_DIR`, or else the directory holding `gr` (`gr:34`). The default directory ships no `.npz` or `.json` [S].
- **Manifest.** The check runs at import (`gr:210-221`).
- **Fallback.** Any gait name with no generator and no capture becomes the **trot table, unreversed** (`gr:436-444`). [S] With no capture, `reach`, `reach_back` and `gallop` are each byte-identical to `trot`.

**Command → reference [R].** `mixed_reference_offset` (`mix:55-109`) runs every step, once for the actor and once for the critic. The same function runs inside the reward with a rewound clock (`mix:141-163`). See §3.

```
 touchdown(4) stride stance_z swing_lift [swing_kf, x/y shift]          captured_reach_gaits.npz          captured_jump_gait.npz
        │                                                              (reach_fwd, reach_back)            (jump, duration_s, touchdown_s)
        ▼ _base_cycle gr:298  (5 stance kf + n swing kf, foot xyz)                 │ + reference_manifest.json   │
        ▼ _interpolate_triangle gr:335 @ (i/n - touchdown[leg]) % 1                │   sha256 + frequency_mult   │
        ▼ + _HIP_OFFSET gr:139 → _ik_single_leg gr:345 (warm start)                │   (import time, gr:210)     │
        ▼                                                                          ▼ _validate_capture (REJECT)  ▼ validate (REJECT)
   (n,12) trot / lift  [NOT clipped]          spider → np.clip        periodic resample (wrap) if N≠n     non-periodic resample
        │                                          │                  reach_back: roll(T[::-1],1)          → np.clip
        └──────────────┬───────────────────────────┴──────────────────────────┬───────────────────────────────┘
                       ▼  build_joint_reference_table gr:386  (unknown gait → trot fallback gr:436)
                       ▼  _reference_table gait:74  → torch.float32, torch.clamp(jnt_range)   ◄── CLIP #2 (all tables)
                       ▼
   command (vx,vy,yaw, post-heading-hold) ─► mixed_reference_offset mix:55
       translating=|vx|≥0.05 · fast=translating∧|vx|≥gallop_speed · freq=f0·(fast? mult_reach:1)
       phase=frac(t·freq); eff=±phase by sign(vx if translating else yaw) · _lookup (linear, floor-mod)
       ref = fast ? (vx≥0 ? reach : reach_back) : (translating ? trot : lift)
       offset = clamp(‖(vx,vy,yaw)‖/0.1, 0, 1) · (ref − default_pose)   ──► actor obs [36+12], critic, reward(t−lead)
                                                                        ──► export.py: trot/gallop(=reach)/lift/gallop_back tables
```

## 2. Capture-socket contract

| File (in `$PUPPER_REFERENCE_DIR`) | Key | Shape / dtype | Constraints (enforced by the loader) | Resampling on load |
|---|---|---|---|---|
| `captured_reach_gaits.npz` (`gr:35`) | `reach_fwd` (gait `reach`, `gr:36`) | `(N, 12)`, any numeric dtype. Cast to float64 by `gr:378`, then float32 in torch. [S] float32 accepted | `N >= 8`; finite; inside joint limits ± 1e-6, else `ValueError` (`gr:372-377`). Loaded with `allow_pickle=False` (`gr:417`), so no object arrays | If `N != n`: periodic linear interpolation. Sample `k` sits at phase `k/N`, row 0 is appended at phase 1 (wrap), and the grid is `i/n` (`gr:421-426`). If `N == n` it passes through [S] |
| same | `reach_back` | same | same | Same resampling, **then** `np.roll(T[::-1], 1, axis=0)` (`gr:428-433`) |
| same | `joint_names` (optional) | string array of 12 | If present it must equal `JOINT_NAMES` exactly (`gr:418-419`). If absent, nothing is checked | n/a |
| same | `metadata_json` (optional) | 0-d str | **Never read** by the loader (written at `boot:160-161`) | n/a |
| `reference_manifest.json` (`gr:210`) | `reference_sha256` | hex str | Must equal the sha256 of the npz bytes, else `ValueError` (`gr:215-216`) [S]. A manifest without an npz raises (`gr:213-214`) [S] | n/a |
| same | `frequency_mult` | float | Must be finite and > 0 (`gr:217-219`). Written to `_GAIT_FREQ_MULT["reach"]` and `["reach_back"]` (`gr:220-221`) [S: 2.0 → both] | n/a |
| same | `version`, `joint_order`, `checks`, `provenance`, `status` | n/a | Written by `boot:162-163`; **ignored by the loader** | n/a |
| `captured_jump_gait.npz` (`gr:40`) | `jump` | `(N, 12)` | Same `_validate_capture` checks (`gr:402`); `joint_names` checked if present (`gr:400-401`) | If `N != n`: **no wrap**, endpoint to endpoint (`gr:403-408`); then `np.clip` (`gr:409`, redundant after validation) |
| same | `duration_s` | scalar | `0.2 < d < 1.44` (`gr:49-50`) | n/a |
| same | `touchdown_s` (optional) | scalar | If absent, defaults to 0.84; must satisfy `0.15 < td <= d` (`gr:60-62`) | n/a |

Other loader behaviour:
- **Missing captures [R/S].** With no `captured_reach_gaits.npz`, `reach` and `reach_back` fall back to the trot table, which is not pre-reversed and is played at a 1× clock (`_GAIT_FREQ_MULT` is `{}`). A backward-fast command then plays the trot reversed in time, the same motion as the slow backward trot. With no jump file, `jump` also falls back to the trot.
- **Npz without a manifest [S].** The capture *is* loaded, but `frequency_mult` silently stays at 1×.
- **Import order [R].** The directory, manifest check and multiplier are module globals (`gr:34-40`, `gr:207-221`). The tables are cached per process (`gr:366`, `gait:49`). `import mjlab.tasks` imports every task package (`src/mjlab/tasks/__init__.py:1-5`), and with it `gr`. So the environment variable must be set **before the registry is imported**; changing it later has no effect. `loco` does this correctly:
  - its top-level imports are stdlib and NumPy only (`loco:6-15`);
  - `main()` sets `os.environ["PUPPER_REFERENCE_DIR"]` from `--reference-dir` (`loco:348-351`) before `train` or `evaluate` lazily import mjlab (for example `loco:174-175`).
  - Any other entry point (the plain `train` script, notebooks) has to export the variable in the shell.
- **Why `reach_back` is pre-reversed [R + S].**
  - The pipeline time-reverses backward commands: `eff = frac(-phase)` (`mix:86`), which `_lookup` floor-mods into `[0, 1)` (`gait:93`).
  - A capture is indexed by the true, forward-running clock: `loco:275` records `phase = mod(t·f, 1)` with no reversal and no lead, and `boot fold` folds on that phase.
  - Write `R(T) = roll(T[::-1], 1)`, so that `R(T)[i] = T[-i mod n]`. Then `lookup(R(T), -φ) = lookup(T, φ)`, and the backward capture plays forward exactly as recorded.
  - [S] `R` is an involution, and `lookup(R(c), frac(-φ)) == lookup(c, φ)` holds for 1237 off-grid phases.
  - **Contract:** store `reach_back` in chronological (recorded) order and do not reverse it yourself.
  - `boot design` stores `R(forward)` as `reach_back` (`boot:375`), meaning "the forward trot run backwards" in chronological order. The loader's `R` then cancels it, so a backward command plays the forward table reversed in time. That is consistent.

## 3. Selection logic (as implemented in `mix:55-109`, mirrored in `exp:344-424`)

```python
cmd = command_manager.get_command("twist")      # POST heading-hold (velocity_command.py:65-68,234-246)
vx, vy, yaw = cmd
translating = |vx| >= _DIR_THRESHOLD (0.05)                        # mix:72
fast        = translating and |vx| >= gallop_speed                 # mix:98 (gallop_speed=0.5 MixedGaits, 1e3 StableGait)
freq  = frequency * (_GAIT_FREQ_MULT.get("reach", 1.0) if fast else 1.0)   # mix:76-81 (only "reach" key is read)
t     = (episode_length_buf [- lead in reward]) * step_dt          # step_dt = 0.004*5 = 0.02 s
phase = frac(t * freq)                                             # mix:82; NOT phase-continuous when fast toggles
dir   = vx if translating else yaw                                 # mix:85
eff   = phase if dir >= 0 else frac(-phase)                        # mix:86 (negative; _lookup floor-mods, gait:90-96)
ref   = (reach if vx >= 0 else reach_back) if fast                 # mix:99,101
        else (trot if translating else lift)                       # mix:100
blend = clamp(||(vx, vy, yaw)|| / blend_speed(0.1), 0, 1)          # mix:104-107 (mixes m/s and rad/s)
offset = blend * (clamp_jnt(ref)[eff] - default_joint_pos)         # mix:103,109 -> actor obs (mix:229), critic (mix:284)
# Reward: mixed_gait_tracking rewinds episode_length_buf by phase_lead_steps (mix:146-160) and
#   returns exp(-mean((q - (default+offset))^2 / GAIT_STD^2))     # mix:161-163
```

Notes on the selection:
- **Standing.** There is no separate stand mode. At a zero command the lift table is selected, but `blend = 0`, so the offset is 0.
- **Small commands.** Below `‖cmd‖ = 0.1` every gait is attenuated linearly.
- **Pure turn or sidestep.** These get the lift table, and its direction follows `sign(yaw)`.
- **Translating commands.** Yaw affects only `blend`.

### Constants

| Name | Value | file:line | Meaning |
|---|---|---|---|
| `GAIT_CYCLE_S` / `GAIT_FREQUENCY` | 0.75 s / 1.333 Hz | `cfg:49-50` | Base phase clock for MixedGaits (passed at `cfg:489`); 37.5 control steps per cycle |
| `STABLE_GAIT_FREQUENCY` | 1.5 Hz | `cfg:149` (used `cfg:530`) | StableGait clock |
| `GAIT_BLEND_SPEED` | 0.1 | `cfg:54` (`cfg:425`) | ‖cmd‖ at which the full reference applies |
| `GAIT_N_SAMPLES` | 100 | `cfg:57` (`cfg:426`) | Table rows; also the resample target for captures |
| `_DIR_THRESHOLD` | 0.05 m/s | `gait:46` | Translating vs turn/sidestep; also exported as `dir_threshold` (`exp:205`) |
| `TROTGALLOP_ONSET` | 0.5 m/s | `cfg:120` (passed `cfg:489`, `cfg:674`) | `gallop_speed` for MixedGaits, so the fast slot is `\|vx\| >= 0.5`; the same onset loosens base-height std |
| `GAIT_GALLOP_SPEED` | 1.0e3 | `cfg:78` (`cfg:530`) | "Never": disables the fast branch (StableGait, single-gait tasks) |
| `_GAIT_FREQ_MULT["reach"]` | **1.0 by default** (dict empty, `gr:209`); else the manifest `frequency_mult` | `gr:207-221`, read at `mix:76` | Fast-slot clock multiplier. Exported as `gallop_freq_mult` (`exp:150-152`) |
| `GAIT_GALLOP_FREQ_MULT` | 2 | `cfg:86` | Single-gait gallop tasks only; **not** used by MixedGaits |
| `boot design --frequency-mult` / `fold --frequency-mult` | 2.0 / **1.0** | `boot:355` / `boot:352` | Value the bootstrap CLI writes into the manifest |
| `MIXED_MAX_LIN_VEL_X` | 1.5 m/s | `cfg:129` (applied `cfg:487`) | The ±1.5 m/s training command range for `lin_vel_x` |
| `TROTGALLOP_MAX_LIN_VEL_X` | 1.0 | `cfg:121` (`cfg:675`) | Range set by `_trot_gallop`, then overridden by `cfg:487` |
| `GAIT_MAX_LIN_VEL_X` / `STABLE_GAIT_MAX_LIN_VEL_X` | 0.7 / 0.7 | `cfg:94` / `cfg:150` | Trot and StableGait command caps |
| `walk_speed_cap` / `run_speed_cap` | 0.49 / 1.5 | `exp:298-299` | Deploy joystick caps, **only in the `jump_slot` block** (MixedGaitsJump). The walk cap sits under the 0.5 onset, so the fast slot needs the run button |
| `MIXED_FELL_OVER_LIMIT_ANGLE` | 45° | `cfg:135` (`cfg:488`) | Termination tilt for MixedGaits |
| `GAIT_GALLOP_HEIGHT_STD` | 10.0 | `cfg:82` | Base-height std when `\|vx\| >= gallop_speed` |
| `GAIT_REWARD_PHASE_LEAD_STEPS` | 1 + 4/5 = 1.8 | `cfg:239-241`; `src/mjlab/tasks/pupper/mdp/latency.py:53` (4 physics steps); `pupper_constants.py:173` (decimation 5) | Reward-only clock rewind (one step for the obs → reward increment, plus 0.8 for action latency). **Effectively 2 in MixedGaits**: see gotcha 2 |
| `GAIT_STD` | `_1`: 0.15, `_2`: 0.3, `_3`: 0.3 | `cfg:164` | Tracking tolerance in rad |
| `GAIT_HEADING_HOLD_KP` / `_CLIP` | 1.0 / 0.3 | `cfg:200-201` (`cfg:367-368`) | Rewrites yaw when \|raw yaw\| < 0.1 and ‖(vx, vy)‖ > 0.05 |
| sim `timestep`, `PUPPER_DECIMATION` | 0.004 s, 5 | `pupper_constants.py:163`, `:173` | `step_dt = 0.02 s` (50 Hz) |
| `_STANCE_Z` / `_SWING_Z` | -0.14 / -0.05 | `gr:186-187` | Defaults when a gait's entry is None (lift = 0.09) |
| IK `lr`, `max_iter`, `tol`, `eps` | 10, 100, 1e-4, 1e-3 | `gr:352` | Gradient-descent IK |
| `_HIP_OFFSET` | FR (.06, -.09), FL (.06, .09), BR (-.11, -.09), BL (-.11, .09) | `gr:139-141` | Body-frame foot origin. This is **not** the FK hip location (`gr:104-132`: ±0.075, ±0.0835 / ±0.0725) |
| Jump duration / touchdown | (0.2, 1.44); default 0.84 | `gr:49`, `gr:60` | Jump socket bounds |

The trot's no-slip speed is `2·0.05 m / (4/6 · 0.75 s) = 0.2 m/s`, which matches the comment at `cfg:118-119`. Above 0.5 m/s with no capture, the feet must slide.

## 4. Surprises and gotchas

1. **Rejection vs clipping is asymmetric.** Captures outside the limits by more than 1e-6 raise an error (`gr:376-377`, `boot:30-31`) [S]. The shipped trot and lift are themselves outside the front-knee limit by up to 0.096 rad and are silently torch-clipped (`gait:86`) [S].
   - `visualize_reference` plays the **unclipped** NumPy table (`visualize_reference.py:66-67`), so what you see is not exactly what is trained.
   - Run `boot validate_reference` on a candidate before training.
2. **The MixedGaits reward lead is an integer.** `mix:149` computes `(buf.float() - 1.8).to(long)`, which truncates toward zero. [S] `[0, 1, 2, 10, 75]` becomes `[-1, 0, 0, 8, 73]`, so the effective lead is 2 steps, not 1.8 (and 1 or 0 for the first two steps). The single-gait reward keeps the float lead (`gait:189`).
3. **`frequency_mult` defaults to 1.** An npz dropped in without a manifest loads at a 1× clock with no warning [S]. `boot fold` writes 1.0 unless you pass `--frequency-mult` (`boot:352`), while `design` defaults to 2.0.
   - Only `_GAIT_FREQ_MULT["reach"]` drives the clock for **both** directions (`mix:76`); the `reach_back` entry is unused in training.
   - The fold's `duration = .75/mult` (`boot:157`) hard-codes 0.75 instead of `GAIT_CYCLE_S`.
4. **`reach_back` ordering.** Store it chronologically; the loader applies `R` (gr:433).
   - Reversing it yourself plays it backwards, with no error.
   - Visualizing `--gait reach_back` shows the *pre-reversed* table, played without the backward reversal, so it looks time-reversed.
   - The periodic resample treats row `N` as row 0: **do not include a duplicate closing row**, or the cycle stalls for one sample. Wrap discontinuity is reported (`boot:37`) but not rejected.
5. **Evaluate needs the same `--reference-dir`.**
   - Without it, the eval env loads the default directory: trot fallback at 1×. The policy then sees a different reference from training, and the run's provenance has no `references` hashes (`loco:77-78`).
   - Eval's recorded `phase` (`loco:271-275`) re-derives `fast` from `TROTGALLOP_ONSET` and the task *name* (`"MixedGaits"`/`"Mystery"`), not from the env's `gallop_speed`. It applies no lead.
   - The exporter likewise reads the multiplier from the module global at export time (`exp:152`), so export must also run under the same directory.
6. **Heading hold drives lift direction.** The reference uses the *emitted* command (`velocity_command.py:65-68`).
   - For a sidestep (`|vx| < 0.05`, raw `|yaw| < 0.1`, `‖(vx, vy)‖ > 0.05`), yaw is the clipped P-correction (`velocity_command.py:237-246`).
   - So the lift phase flips between φ and −φ whenever the heading error changes sign, despite the docstring's "cannot chatter" (`mix:30-31`).
7. **The phase jumps at the 0.5 m/s boundary.** `phase = frac(t·f·mult)` is recomputed from absolute episode time (`mix:82`). Crossing the onset, or flipping direction, jumps the phase discontinuously. This is harmless in training (the command is piecewise constant) but visible on a joystick.
8. **Joint naming.** `DEFAULT_POSE`'s comment says "(abduction, hip, knee)" (`pupper_constants.py:47`), but `mix:169-171` (by FK perturbation) and the tables [S: `_2` peak-to-peak is about 0.15 or less, `_1` is 1.2 to 1.3] show that `_2` is roll/abduction and `_1` is fore-aft. The `GAIT_STD` comment (`cfg:162-163`) inherits the wrong label.
9. **Corrections to `B_repo_map.md` §3.**
   - "Manifest required if present": the manifest is *optional*. An npz with no manifest loads at 1× [S]; a manifest with no npz raises.
   - `touchdown_s` is optional, with a 0.84 fallback (`gr:60`).
   - Resampling targets `n_samples` and happens only when `N != n` (not always "onto 100").
   - The F jump-resample lines are `gr:403-408`, not U:367-372.
   - `run_speed_cap` and `walk_speed_cap` exist only in `jump_slot_metadata`, not in the plain MixedGaits export.
   - It misses the front-knee overshoot of trot and lift, the integer reward lead, and the heading-hold interaction.
   - Its other F line references and values checked out.
