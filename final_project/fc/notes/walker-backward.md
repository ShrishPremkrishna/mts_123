# Backward walking cap (study, 2026-11-28; proposal only, nothing changed)

**Answer:** the 0.3 m/s backward limit is a config value, `max_vx_backward: 0.3`, which CS123 added to the walker
`neural_controller_default`. The policy itself was trained on vx ±1.5, backward included. Raising the limit needs no
rebuild and no retraining: it is a per-start YAML parameter. Plan: go to **0.45** first (same trot gait, ~1.5× today),
then 0.6–0.75 (the backward fast gait) only after a sim check and floor step tests.

## Where backward vx gets clipped (laptop → walker)
1. `brain.shape` (laptop): `-min(max_vx, max_vx_back)`. `max_vx_back` is 0.3 (`brain.py:64`), a live param.
2. `drive_bridge.py`: symmetric ±`max_vx` (= the start's speed cap, at most 1.5). Not the limit.
3. `cmd_vel_mux` (mts `ros2_ws/src/cmd_vel_mux`): deadband 0.05 and a 500 ms timeout. It does not clip. The gamepad's
   teleop scale is 1.5 (affects only the stick).
4. `neural_controller.cpp:652-660` (mts): `command_clip` from the policy JSON (vx [-1.5, 1.5]), then
   `max_vx_forward` 0.49 / **`max_vx_backward` 0.3**, set at `launch/config.yaml:322-323`. In mts's monorepo
   `config.yaml` is a local modification (`git status` M, the "CS123 fall-26 build"), and the same is in our
   `pupper_gait_deploy/config.yaml`. The installed file is a symlink to it. The parameter default is 0 (off).
5. Jump-slot cap (`:691-697`): ±`speed_cap` on vx and vy. `stack_overlay.py --speed-cap` sets it to the cap (play
   preset 1.0), so it does not bind backward. Effective backward = min(`max_vx_backward`, `speed_cap`, brain).
- `stack_overlay.py` changes `max_vx_forward` and `speed_cap` only, and states that `max_vx_backward` stays unchanged.
  The L1+R1 partner `_default_alt` has no `max_vx_*` at all.

## The policy's trained range
- `default_policy.json` (sha `ecf3a4ab8ea3`, the same on mts and in `pupper_gait_deploy/policies/`) contains
  `command_clip.vx [-1.5, 1.5]`, `gallop_speed 0.5` and a `gallop_back_table`. Only the MixedGaits exporter writes
  that table (`pupper-mjlab .../pupper_gait_env_cfg.py: pupper_mixed_gaits_env_cfg`, `lin_vel_x = ±MIXED_MAX_LIN_VEL_X` = ±1.5).
  The table is the captured gait that run `al7sdood` actually walked at **−1.5 m/s** (changelog: "Forward and
  backward are separate captures"). The exact checkpoint run is not documented (see fast-gait.md).
- |vx| < 0.5: the trot, played time-reversed for backward. |vx| ≥ 0.5: the backward fast gait, a different motion with
  2.5× cadence. So 0.3 → 0.45 only speeds up the trot. Crossing 0.5 switches gait (`gait_reference.hpp:95-118`).
- Evidence so far: Nov 27 keeper, −0.75 commanded gave ~0.21 m/s (clipped to 0.3, gain ~0.7). Forward sim at 0.3
  gave 0.26. Forward fast gait on hardware was clean at 1.0 and 1.2 commanded, and fell at 1.4 (STATUS Nov 27).
  **No backward data above 0.3** exists in sim or on hardware.

## Proposed change (for the owners of stack_overlay / fcctl / app / brain)
- `stack_overlay.py --back-cap X` (0.30..1.50): set `max_vx_backward` on `STOCK_WALKERS` (+ the u6 baseline) and add
  it to the verify-exact-edits list and `stack.json`. Plumb it through fcctl `--back-cap` and preset `_stack.back_cap`.
  Brain: `max_vx_back` = the same value. Keep `back_cap ≤ speed_cap` (the jump-slot cap clips backward too).
- Do not hand-edit mts's `config.yaml`. It is shared by every start, `provision --check` only tests that the file
  exists, and it would hide the change from the run record.
- Value: **0.45** for the demo, unless the floor ladder below passes at 0.6/0.75. Expected walking speed ≈ 0.32 m/s
  at gain 0.7. Retune `gk_asym_m` (the keeper's slow-side offset) after the change.

## Risks
- Backward falls are tail-first and blind. The fallen cutoff and ImuHealth tilt stop still apply. At 0.5 the reference
  switches tables instantly (no cross-fade; training used the same switch). The worst case is a keeper reversing
  forward→back across it.
- The fast gait wobbles ±0.5 rad/s in yaw (forward, sim), so expect the same backward. Stopping from higher speed
  takes longer near the goal line. Check that `_fence_guard` budgets backward motion toward the back fence.

## Test plan (operator; one robot, preset `slow` first, spotter, R3 ready)
0. Sim first (laptop): the fast-gait harness from fast-gait.md is not in the repo. `fc/robot/sim/liedown_sim.py`'s
   model plus the production inference can run vx −0.3/−0.45/−0.6/−0.75/−1.0 (8 runs, nominal + harsh) and
   reversals +0.5 → −0.6. Pass: 0 falls, tilt < 15°.
1. Stand: start with the new cap 0.45, Ready on the stand, then a short backward `drive` with legs in the air: no fault.
2. Floor, ≥ 1.5 m clear behind: `ctl set max_vx_back=0.45`, then `ctl test step vx=-0.3 seconds=1.5` (baseline) and
   `vx=-0.45`, 3 reps each. Read `gain_lin`, `heading_bias_deg` and tilt from the run summary.
3. Only if step 2 is clean: cap 0.6, steps −0.55, −0.6, −0.75, then fwd 0.5 → back −0.6 reversals (`goalie` test).
- Stop at the first stumble, tilt > 15°, heading change > 20° within a step, or fallen cutoff. Pass: 0 falls in
  3 reps per speed and `gain_lin` within ±10%.
